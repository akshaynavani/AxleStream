#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <random>
#include <stdexcept>

#include "axle/exporter.hpp"

namespace fs = std::filesystem;

namespace axle {

namespace {

std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 2);
    for (const char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof buf, "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

/// Small append-only JSON object builder; avoids pulling in a JSON library for a hot path.
class JsonObj {
public:
    JsonObj() { s_.reserve(1024); s_ += '{'; }

    JsonObj& str(const char* k, const std::string& v) {
        key(k);
        s_ += '"';
        s_ += json_escape(v);
        s_ += '"';
        return *this;
    }
    JsonObj& i64(const char* k, long long v) {
        key(k);
        s_ += std::to_string(v);
        return *this;
    }
    JsonObj& u64(const char* k, unsigned long long v) {
        key(k);
        s_ += std::to_string(v);
        return *this;
    }
    JsonObj& num(const char* k, double v, int precision = 4) {
        key(k);
        if (!std::isfinite(v)) {
            s_ += "null";
        } else {
            char buf[48];
            std::snprintf(buf, sizeof buf, "%.*f", precision, v);
            s_ += buf;
        }
        return *this;
    }
    /// Emits null for the "not available" sentinel used by processed types (< 0).
    JsonObj& opt(const char* k, double v, int precision = 4) {
        if (v < 0) {
            key(k);
            s_ += "null";
            return *this;
        }
        return num(k, v, precision);
    }
    JsonObj& boolean(const char* k, bool v) {
        key(k);
        s_ += v ? "true" : "false";
        return *this;
    }
    JsonObj& raw(const char* k, const std::string& json) {
        key(k);
        s_ += json;
        return *this;
    }
    std::string done() {
        s_ += '}';
        return std::move(s_);
    }

private:
    void key(const char* k) {
        if (!first_) s_ += ',';
        first_ = false;
        s_ += '"';
        s_ += k;
        s_ += "\":";
    }
    std::string s_;
    bool first_{true};
};

std::string utc_date(Nanos wall) {
    const std::time_t secs = static_cast<std::time_t>(wall / 1'000'000'000);
    std::tm tm{};
    gmtime_r(&secs, &tm);
    char buf[16];
    std::strftime(buf, sizeof buf, "%Y-%m-%d", &tm);
    return buf;
}

}  // namespace

std::string make_run_id() {
    const std::time_t now = std::time(nullptr);
    std::tm tm{};
    gmtime_r(&now, &tm);
    char ts[32];
    std::strftime(ts, sizeof ts, "%Y%m%dT%H%M%SZ", &tm);
    std::random_device rd;
    char suffix[8];
    std::snprintf(suffix, sizeof suffix, "%04x", rd() & 0xFFFF);
    return std::string(ts) + "-" + suffix;
}

JsonlExporter::JsonlExporter(ExportConfig cfg) : cfg_(std::move(cfg)) {
    if (cfg_.run_id.empty()) cfg_.run_id = make_run_id();
    fs::create_directories(cfg_.out_dir);
}

JsonlExporter::~JsonlExporter() { close(); }

std::string JsonlExporter::to_json(const FusedFrame& f, const ExportConfig& cfg, Nanos e2e_latency_ns,
                                   bool over_budget) {
    const auto& l = f.lidar;
    std::string obstacles = "[";
    const std::size_t n_obs = std::min(cfg.max_obstacles, l.obstacles.size());
    for (std::size_t i = 0; i < n_obs; ++i) {
        const auto& o = l.obstacles[i];
        if (i) obstacles += ',';
        obstacles += JsonObj()
                         .num("x", o.centroid.x, 2)
                         .num("y", o.centroid.y, 2)
                         .num("z", o.centroid.z, 2)
                         .num("len", o.max.x - o.min.x, 2)
                         .num("wid", o.max.y - o.min.y, 2)
                         .num("hgt", o.max.z - o.min.z, 2)
                         .num("range_m", o.range_m, 2)
                         .u64("points", o.num_points)
                         .done();
    }
    obstacles += ']';

    JsonObj j;
    j.str("vehicle_id", cfg.vehicle_id)
        .str("run_id", cfg.run_id)
        .u64("seq", f.seq)
        .i64("sensor_ts_ns", f.sensor_ts_ns)
        .num("e2e_latency_ms", ns_to_ms(e2e_latency_ns), 4)
        .num("lidar_proc_ms", ns_to_ms(l.processed_ns - l.ingest_ns), 4)
        .boolean("over_budget", over_budget)
        .u64("raw_points", l.raw_points)
        .u64("filtered_points", l.filtered_points)
        .u64("ground_points", l.ground_points)
        .u64("num_obstacles", l.obstacles.size())
        .opt("nearest_obstacle_m", l.nearest_obstacle_m, 2)
        .opt("ttc_s", f.time_to_collision_s, 3)
        .boolean("imu_valid", f.imu_valid)
        .boolean("vehicle_valid", f.vehicle_valid);

    if (f.imu_valid) {
        j.num("roll_rad", f.imu.roll_rad, 5)
            .num("pitch_rad", f.imu.pitch_rad, 5)
            .num("yaw_rad", f.imu.yaw_rad, 5)
            .num("yaw_rate_rps", f.imu.yaw_rate_rps, 5)
            .num("long_accel_mps2", f.imu.long_accel_mps2, 3)
            .num("lat_accel_mps2", f.imu.lat_accel_mps2, 3)
            .num("accel_norm_mps2", f.imu.accel_norm_mps2, 3);
    }
    if (f.vehicle_valid) {
        const auto& v = f.vehicle;
        j.num("speed_mps", v.speed_mps, 3)
            .num("steering_deg", v.steering_deg, 2)
            .num("engine_rpm", v.engine_rpm, 1)
            .num("throttle_pct", v.throttle_pct, 1)
            .num("brake_bar", v.brake_bar, 2)
            .u64("gear", v.gear)
            .u64("can_decode_errors", v.decode_errors);
    }
    j.raw("obstacles", obstacles);
    return j.done();
}

void JsonlExporter::open_new_file(Nanos sensor_ts_ns) {
    const fs::path dir = fs::path(cfg_.out_dir) / "frames" / ("vehicle_id=" + cfg_.vehicle_id) /
                         ("date=" + utc_date(sensor_ts_ns > 0 ? sensor_ts_ns : wall_ns()));
    fs::create_directories(dir);
    char name[128];
    std::snprintf(name, sizeof name, "part-%s-%05zu.jsonl", cfg_.run_id.c_str(), file_index_++);
    final_path_ = (dir / name).string();
    tmp_path_ = final_path_ + ".inprogress";
    file_ = std::fopen(tmp_path_.c_str(), "w");
    if (!file_) throw std::runtime_error("cannot open export file " + tmp_path_);
    // Large stdio buffer: one fwrite per ~64 KiB keeps syscalls off the export thread.
    std::setvbuf(file_, nullptr, _IOFBF, 1 << 16);
    records_in_file_ = 0;
    file_opened_mono_ = now_ns();
}

void JsonlExporter::finish_file() {
    if (!file_) return;
    std::fclose(file_);
    file_ = nullptr;
    if (records_in_file_ == 0) {
        fs::remove(tmp_path_);
        return;
    }
    fs::rename(tmp_path_, final_path_);
    ++files_closed_;
}

void JsonlExporter::rotate_if_needed() {
    if (!file_) return;
    const bool full = records_in_file_ >= cfg_.records_per_file;
    const bool old = static_cast<double>(now_ns() - file_opened_mono_) / 1e9 >= cfg_.max_file_age_s;
    if (full || old) finish_file();
}

void JsonlExporter::write(const FusedFrame& f, Nanos e2e_latency_ns, bool over_budget) {
    rotate_if_needed();
    if (!file_) open_new_file(f.sensor_ts_ns);
    const std::string line = to_json(f, cfg_, e2e_latency_ns, over_budget);
    std::fwrite(line.data(), 1, line.size(), file_);
    std::fputc('\n', file_);
    ++records_in_file_;
    ++total_records_;
}

void JsonlExporter::flush() {
    if (file_) std::fflush(file_);
}

void JsonlExporter::close() { finish_file(); }

void write_run_summary(const std::string& path, const std::string& json) {
    fs::create_directories(fs::path(path).parent_path());
    std::ofstream out(path, std::ios::trunc);
    out << json << '\n';
}

}  // namespace axle
