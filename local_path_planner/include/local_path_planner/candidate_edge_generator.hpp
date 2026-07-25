#pragma once

#include <vector>

#include "local_path_planner/planner_types.hpp"

namespace local_path_planner
{

struct CandidateEdgeGeneratorConfig
{
    bool use_delaunay = true;
    bool fallback_to_pairwise = true;
    int min_delaunay_candidates = 3;
    double min_width = 1.5;
    double max_width = 4.5;
    double expected_width = 2.5;
    double max_longitudinal_offset = 1.5;
    double min_midpoint_x = 0.2;
    double max_reference_distance = 2.0;
    double width_cost_weight = 1.0;
    double longitudinal_cost_weight = 0.8;
    double confidence_cost_weight = 0.8;
    double reference_cost_weight = 1.0;
    int max_edges_per_cone = 5;
};

class CandidateEdgeGenerator
{
public:
    explicit CandidateEdgeGenerator(
        const CandidateEdgeGeneratorConfig& config = CandidateEdgeGeneratorConfig());

    std::vector<CandidateEdge> generate(
        const std::vector<ConePoint>& cones,
        const std::vector<Point2D>& reference_path = std::vector<Point2D>()) const;

private:
    CandidateEdgeGeneratorConfig config_;
};

}  // namespace local_path_planner
