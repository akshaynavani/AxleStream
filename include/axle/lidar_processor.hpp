#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "axle/types.hpp"

namespace axle {

/// Per-frame LiDAR processing:
///   1. range crop (drops ego-body returns and far, sparse points)
///   2. voxel-grid downsample (centroid per occupied voxel)
///   3. ground segmentation on a coarse 2D grid (local minimum height per cell)
///   4. Euclidean-style clustering of non-ground voxels via 8-connected occupancy grid
///
/// All scratch buffers are members and reused between frames, so steady-state processing
/// performs no heap allocation beyond the output obstacle list. One instance per thread.
class LidarProcessor {
public:
    struct Config {
        float min_range_m = 1.5f;
        float max_range_m = 60.f;
        float max_height_m = 2.5f;         // above sensor; ignore overhead returns
        float voxel_size_m = 0.2f;
        float ground_cell_m = 1.0f;
        float ground_tolerance_m = 0.2f;   // height above local min still counted as ground
        float sensor_height_m = 1.8f;      // expected ground plane at z = -sensor_height
        float ground_max_offset_m = 0.6f;  // cell min must be near the expected plane
        float cluster_cell_m = 0.5f;
        std::uint32_t min_cluster_points = 3;
        std::uint32_t max_cluster_points = 20000;
    };

    LidarProcessor();
    explicit LidarProcessor(Config cfg);

    LidarResult process(const LidarFrame& frame);

    const Config& config() const noexcept { return cfg_; }

private:
    struct VoxelAccum {
        float sx, sy, sz, si;
        std::uint32_t n;
    };

    void voxelize(const LidarFrame& frame);
    void segment_ground(LidarResult& out);
    void cluster(LidarResult& out);

    Config cfg_;
    int grid_dim_;  // cells per side for the cluster grid

    std::unordered_map<std::uint64_t, VoxelAccum> voxels_;
    std::vector<LidarPoint> filtered_;
    std::vector<LidarPoint> obstacles_pts_;

    std::unordered_map<std::uint64_t, float> ground_min_;

    std::vector<std::int32_t> cell_head_;  // first point index in cell, -1 if empty
    std::vector<std::int32_t> next_in_cell_;
    std::vector<std::int32_t> cell_label_;
    std::vector<std::int32_t> touched_cells_;
    std::vector<std::int32_t> bfs_;
};

}  // namespace axle
