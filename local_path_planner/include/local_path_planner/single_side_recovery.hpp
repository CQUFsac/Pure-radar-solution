#pragma once

#include <string>
#include <vector>

#include "local_path_planner/planner_types.hpp"

namespace local_path_planner
{

struct SingleSideRecoveryConfig
{
    bool enabled = true;
    double expected_track_width = 3.0;
    double min_boundary_offset = 0.65;
    double max_boundary_offset = 3.5;
    double max_center_reference_distance = 1.25;
    double min_forward_x = 0.05;
    double max_forward_x = 12.0;
    double min_point_spacing = 0.25;
    int min_virtual_points = 1;
    double confidence_scale = 0.55;
};

struct SingleSideRecoveryResult
{
    bool success = false;
    std::string reason;
    std::vector<Point2D> centerline;
    int virtual_point_count = 0;
    double confidence = 0.0;
};

// When one boundary cone has no opposite cone, infer a temporary centre point
// from the previous centreline normal. This is only a short degraded fallback.
class SingleSideRecovery
{
public:
    explicit SingleSideRecovery(
        const SingleSideRecoveryConfig& config = SingleSideRecoveryConfig());

    SingleSideRecoveryResult recover(
        const std::vector<ConePoint>& cones,
        const std::vector<Point2D>& reference_path,
        double reference_confidence) const;

private:
    SingleSideRecoveryConfig config_;
};

}  // namespace local_path_planner
