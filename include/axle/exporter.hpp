#pragma once

#include <cstdio>
#include <memory>
#include <string>

#include "axle/config.hpp"
#include "axle/types.hpp"

namespace axle {

/// Terminal stage consumer for fused frames.
class FrameSink {
public:
    virtual ~FrameSink() = default;
    virtual void write(const FusedFrame& f, Nanos e2e_latency_ns, bool over_budget) = 0;
    virtual void flush() {}
    virtual void close() {}
};

/// Discards everything (benchmarks, `--no-export`).
class NullSink final : public FrameSink {
public:
    void write(const FusedFrame&, Nanos, bool) override {}
};

/// Writes one JSON object per fused frame into rotating JSON Lines files laid out for
/// Databricks Auto Loader / Spark partition discovery:
///
///   <out_dir>/frames/vehicle_id=<id>/date=<YYYY-MM-DD>/part-<run>-<n>.jsonl
///
/// Files are written as `*.jsonl.inprogress` and atomically renamed when rotated/closed,
/// so an uploader polling the directory only ever sees complete files.
class JsonlExporter final : public FrameSink {
public:
    explicit JsonlExporter(ExportConfig cfg);
    ~JsonlExporter() override;

    void write(const FusedFrame& f, Nanos e2e_latency_ns, bool over_budget) override;
    void flush() override;
    void close() override;

    const std::string& run_id() const noexcept { return cfg_.run_id; }
    std::size_t files_written() const noexcept { return files_closed_; }
    std::size_t records_written() const noexcept { return total_records_; }

    /// Serialise one frame (exposed for tests).
    static std::string to_json(const FusedFrame& f, const ExportConfig& cfg, Nanos e2e_latency_ns,
                               bool over_budget);

private:
    void open_new_file(Nanos sensor_ts_ns);
    void rotate_if_needed();
    void finish_file();

    ExportConfig cfg_;
    std::FILE* file_{nullptr};
    std::string tmp_path_;
    std::string final_path_;
    std::size_t file_index_{0};
    std::size_t records_in_file_{0};
    std::size_t files_closed_{0};
    std::size_t total_records_{0};
    Nanos file_opened_mono_{0};
};

/// Writes a small JSON run summary (latency percentiles, drop counts) next to the frames.
void write_run_summary(const std::string& path, const std::string& json);

std::string make_run_id();

}  // namespace axle
