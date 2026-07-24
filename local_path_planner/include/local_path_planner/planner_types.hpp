#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace local_path_planner
{

struct Point2D
{
    double x = 0.0;
    double y = 0.0;
};

struct ConePoint
{
    std::uint32_t id = 0U;
    Point2D position;
    double confidence = 0.0;
    bool confirmed = false;
};

struct CandidateEdge
{
    std::size_t first_cone_index = 0U;
    std::size_t second_cone_index = 0U;
    std::uint32_t first_cone_id = 0U;
    std::uint32_t second_cone_id = 0U;
    Point2D midpoint;
    double width = 0.0;
    double cost = 0.0;
    double confidence = 0.0;
};

struct SearchResult
{
    bool success = false;
    std::string reason;
    std::vector<std::size_t> selected_edge_indices;
    std::vector<Point2D> midpoints;
    int gap_bridges = 0;
    double cost = 0.0;
    double confidence = 0.0;
};

struct PathPoint
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double curvature = 0.0;
    double confidence = 0.0;
};

struct PlannerResult
{
    bool valid = false;
    std::string reason;
    std::vector<CandidateEdge> candidate_edges;
    SearchResult search_result;
    std::vector<PathPoint> path;
    double confidence = 0.0;
};

}  // namespace local_path_planner
