#pragma once

#include <string>
#include <vector>

#include "local_path_planner/planner_types.hpp"

namespace local_path_planner
{

struct PathValidatorConfig
{
    int min_path_points = 8;
    double min_path_length = 2.0;
    double min_forward_progress = 2.0;
    double min_path_confidence = 0.35;
    double max_point_spacing = 0.6;
    double max_heading_change = 0.79;
    double max_abs_curvature = 1.5;
    double max_curvature_change = 1.2;
    double min_cone_clearance = 0.35;
};

class PathValidator
{
public:
    explicit PathValidator(const PathValidatorConfig& config = PathValidatorConfig());

    bool validate(
        const std::vector<PathPoint>& path,
        const std::vector<ConePoint>& cones,
        std::string& reason) const;

private:
    PathValidatorConfig config_;
};

}  // namespace local_path_planner
