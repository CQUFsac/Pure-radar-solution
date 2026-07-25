#pragma once

#include <vector>

#include "local_path_planner/planner_types.hpp"

namespace local_path_planner
{

struct PathSmootherConfig
{
    bool use_cubic_spline = true;
    double min_input_spacing = 0.1;
    double sample_spacing = 0.2;
    double spline_tension = 0.25;
    double max_spline_deviation = 0.45;
    int smoothing_passes = 1;
};

class PathSmoother
{
public:
    explicit PathSmoother(const PathSmootherConfig& config = PathSmootherConfig());

    std::vector<PathPoint> smooth(
        const std::vector<Point2D>& points,
        double confidence) const;

private:
    PathSmootherConfig config_;
};

}  // namespace local_path_planner
