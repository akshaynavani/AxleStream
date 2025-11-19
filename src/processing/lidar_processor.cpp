#include "axle/lidar_processor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace axle {

namespace {

// Pack three signed 21-bit cell coordinates into one 64-bit key.
inline std::uint64_t pack3(std::int32_t x, std::int32_t y, std::int32_t z) noexcept {
    constexpr std::uint64_t kMask = (1u << 21) - 1;
    return ((static_cast<std::uint64_t>(x) & kMask) << 42) | ((static_cast<std::uint64_t>(y) & kMask) << 21) |
           (static_cast<std::uint64_t>(z) & kMask);
}

inline std::uint64_t pack2(std::int32_t x, std::int32_t y) noexcept {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) | static_cast<std::uint32_t>(y);
}

inline std::int32_t cell_of(float v, float size) noexcept {
    return static_cast<std::int32_t>(std::floor(v / size));
}

}  // namespace

LidarProcessor::LidarProcessor() : LidarProcessor(Config{}) {}

LidarProcessor::LidarProcessor(Config cfg) : cfg_(cfg) {
    grid_dim_ = static_cast<int>(std::ceil(2.f * cfg_.max_range_m / cfg_.cluster_cell_m)) + 2;
    cell_head_.assign(static_cast<std::size_t>(grid_dim_) * static_cast<std::size_t>(grid_dim_), -1);
    cell_label_.assign(cell_head_.size(), -1);
    voxels_.reserve(16384);
    ground_min_.reserve(8192);
}

LidarResult LidarProcessor::process(const LidarFrame& frame) {
    LidarResult out;
    out.seq = frame.seq;
    out.sensor_ts_ns = frame.sensor_ts_ns;
    out.ingest_ns = frame.ingest_ns;
    out.raw_points = static_cast<std::uint32_t>(frame.points.size());

    voxelize(frame);
    out.filtered_points = static_cast<std::uint32_t>(filtered_.size());
    segment_ground(out);
    cluster(out);

    out.processed_ns = now_ns();
    return out;
}

void LidarProcessor::voxelize(const LidarFrame& frame) {
    voxels_.clear();
    const float min_r2 = cfg_.min_range_m * cfg_.min_range_m;
    const float max_r2 = cfg_.max_range_m * cfg_.max_range_m;
    const float inv = 1.f / cfg_.voxel_size_m;

    for (const auto& p : frame.points) {
        const float r2 = p.x * p.x + p.y * p.y;
        if (r2 < min_r2 || r2 > max_r2 || p.z > cfg_.max_height_m || !std::isfinite(r2)) continue;
        const auto key = pack3(static_cast<std::int32_t>(std::floor(p.x * inv)),
                               static_cast<std::int32_t>(std::floor(p.y * inv)),
                               static_cast<std::int32_t>(std::floor(p.z * inv)));
        auto& v = voxels_[key];  // value-initialised to zero on first touch
        v.sx += p.x;
        v.sy += p.y;
        v.sz += p.z;
        v.si += p.intensity;
        ++v.n;
    }

    filtered_.clear();
    filtered_.reserve(voxels_.size());
    for (const auto& [key, v] : voxels_) {
        const float n = static_cast<float>(v.n);
        filtered_.push_back({v.sx / n, v.sy / n, v.sz / n, v.si / n});
    }
    // Deterministic order regardless of hash-map iteration (helps reproducible output).
    std::sort(filtered_.begin(), filtered_.end(), [](const LidarPoint& a, const LidarPoint& b) {
        return a.x != b.x ? a.x < b.x : (a.y != b.y ? a.y < b.y : a.z < b.z);
    });
}

void LidarProcessor::segment_ground(LidarResult& out) {
    ground_min_.clear();
    for (const auto& p : filtered_) {
        const auto key = pack2(cell_of(p.x, cfg_.ground_cell_m), cell_of(p.y, cfg_.ground_cell_m));
        auto it = ground_min_.find(key);
        if (it == ground_min_.end()) {
            ground_min_.emplace(key, p.z);
        } else if (p.z < it->second) {
            it->second = p.z;
        }
    }

    const float expected = -cfg_.sensor_height_m;
    obstacles_pts_.clear();
    std::uint32_t ground = 0;
    for (std::size_t i = 0; i < filtered_.size(); ++i) {
        const auto& p = filtered_[i];
        const float cell_min =
            ground_min_[pack2(cell_of(p.x, cfg_.ground_cell_m), cell_of(p.y, cfg_.ground_cell_m))];
        const bool plausible_ground_cell = std::abs(cell_min - expected) < cfg_.ground_max_offset_m;
        if (plausible_ground_cell && p.z - cell_min < cfg_.ground_tolerance_m) {
            ++ground;
        } else {
            obstacles_pts_.push_back(p);
        }
    }
    out.ground_points = ground;
}

void LidarProcessor::cluster(LidarResult& out) {
    const auto n = obstacles_pts_.size();
    next_in_cell_.assign(n, -1);
    touched_cells_.clear();

    const float origin = -cfg_.max_range_m - cfg_.cluster_cell_m;  // grid covers [-R, R]
    auto cell_index = [&](const LidarPoint& p) -> std::int32_t {
        const auto cx = static_cast<std::int32_t>((p.x - origin) / cfg_.cluster_cell_m);
        const auto cy = static_cast<std::int32_t>((p.y - origin) / cfg_.cluster_cell_m);
        if (cx < 0 || cy < 0 || cx >= grid_dim_ || cy >= grid_dim_) return -1;
        return cy * grid_dim_ + cx;
    };

    for (std::size_t i = 0; i < n; ++i) {
        const auto c = cell_index(obstacles_pts_[i]);
        if (c < 0) continue;
        auto& head = cell_head_[static_cast<std::size_t>(c)];
        if (head < 0) touched_cells_.push_back(c);
        next_in_cell_[i] = head;
        head = static_cast<std::int32_t>(i);
    }

    float nearest = std::numeric_limits<float>::max();
    std::int32_t label = 0;
    for (const auto start : touched_cells_) {
        if (cell_label_[static_cast<std::size_t>(start)] >= 0) continue;

        Obstacle ob;
        ob.min = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                  std::numeric_limits<float>::max()};
        ob.max = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                  std::numeric_limits<float>::lowest()};
        double sx = 0, sy = 0, sz = 0;
        std::uint32_t count = 0;

        bfs_.clear();
        bfs_.push_back(start);
        cell_label_[static_cast<std::size_t>(start)] = label;
        for (std::size_t q = 0; q < bfs_.size(); ++q) {
            const auto c = bfs_[q];
            for (auto i = cell_head_[static_cast<std::size_t>(c)]; i >= 0; i = next_in_cell_[static_cast<std::size_t>(i)]) {
                const auto& p = obstacles_pts_[static_cast<std::size_t>(i)];
                ob.min = {std::min(ob.min.x, p.x), std::min(ob.min.y, p.y), std::min(ob.min.z, p.z)};
                ob.max = {std::max(ob.max.x, p.x), std::max(ob.max.y, p.y), std::max(ob.max.z, p.z)};
                sx += p.x;
                sy += p.y;
                sz += p.z;
                ++count;
            }
            const int cx = c % grid_dim_, cy = c / grid_dim_;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int nx = cx + dx, ny = cy + dy;
                    if (nx < 0 || ny < 0 || nx >= grid_dim_ || ny >= grid_dim_) continue;
                    const auto nc = static_cast<std::size_t>(ny * grid_dim_ + nx);
                    if (cell_head_[nc] >= 0 && cell_label_[nc] < 0) {
                        cell_label_[nc] = label;
                        bfs_.push_back(static_cast<std::int32_t>(nc));
                    }
                }
            }
        }
        ++label;

        if (count < cfg_.min_cluster_points || count > cfg_.max_cluster_points) continue;
        ob.num_points = count;
        ob.centroid = {static_cast<float>(sx / count), static_cast<float>(sy / count),
                       static_cast<float>(sz / count)};
        // Range to the closest face of the box, measured in the ground plane.
        const float px = std::clamp(0.f, ob.min.x, ob.max.x);
        const float py = std::clamp(0.f, ob.min.y, ob.max.y);
        ob.range_m = std::sqrt(px * px + py * py);
        nearest = std::min(nearest, ob.range_m);
        out.obstacles.push_back(ob);
    }

    // Only occupied cells are ever labelled and all of them are in touched_cells_, so
    // resetting just those leaves the grid clean for the next frame in O(touched).
    for (const auto c : touched_cells_) {
        cell_head_[static_cast<std::size_t>(c)] = -1;
        cell_label_[static_cast<std::size_t>(c)] = -1;
    }

    std::sort(out.obstacles.begin(), out.obstacles.end(),
              [](const Obstacle& a, const Obstacle& b) { return a.range_m < b.range_m; });
    out.nearest_obstacle_m = out.obstacles.empty() ? -1.f : nearest;
}

}  // namespace axle
