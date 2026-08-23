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
    bool hard_reference_gate = true;
    double width_cost_weight = 1.0;
    double longitudinal_cost_weight = 0.8;
    double confidence_cost_weight = 0.8;
    double reference_cost_weight = 1.0;
    bool use_semantic_pairing = false;
    bool allow_orange_cross_edges = false;
    bool enforce_semantic_one_to_one = true;
    double unknown_semantic_penalty = 0.6;
    bool use_semantic_boundary_recovery = false;
    bool semantic_boundary_requires_reference = true;
    double expected_boundary_spacing = 2.8;
    double min_boundary_link_distance = 0.4;
    double max_boundary_link_distance = 5.0;
    double max_boundary_heading_error = 0.96;
    double max_boundary_start_heading_error = 0.61;
    double min_boundary_reference_offset = 0.5;
    double max_boundary_reference_offset = 4.0;
    double max_boundary_center_reference_distance = 1.8;
    double max_boundary_reference_progress_step = 7.0;
    double max_pair_reference_progress_difference = 3.0;
    double boundary_virtual_cost = 0.35;
    int max_boundary_edges_per_cone = 2;
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
