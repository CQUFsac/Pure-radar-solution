#include "local_path_planner/candidate_edge_generator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace local_path_planner
{
namespace
{

struct EdgeKey
{
    EdgeKey(std::size_t first_value, std::size_t second_value)
        : first(std::min(first_value, second_value)),
          second(std::max(first_value, second_value))
    {
    }

    bool operator<(const EdgeKey& other) const
    {
        return first < other.first ||
            (first == other.first && second < other.second);
    }

    std::size_t first;
    std::size_t second;
};

struct Triangle
{
    std::size_t first = 0U;
    std::size_t second = 0U;
    std::size_t third = 0U;
    Point2D center;
    double radius_squared = 0.0;
};

struct ReferenceInfo
{
    bool valid = false;
    Point2D tangent{1.0, 0.0};
    double distance = 0.0;
};

bool makeTriangle(
    const std::vector<Point2D>& points,
    std::size_t first,
    std::size_t second,
    std::size_t third,
    Triangle& output)
{
    const Point2D& a = points[first];
    const Point2D& b = points[second];
    const Point2D& c = points[third];
    const double denominator = 2.0 *
        (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    if (std::abs(denominator) < 1.0e-10)
    {
        return false;
    }

    const double a_squared = a.x * a.x + a.y * a.y;
    const double b_squared = b.x * b.x + b.y * b.y;
    const double c_squared = c.x * c.x + c.y * c.y;
    output.first = first;
    output.second = second;
    output.third = third;
    output.center.x =
        (a_squared * (b.y - c.y) + b_squared * (c.y - a.y) +
         c_squared * (a.y - b.y)) /
        denominator;
    output.center.y =
        (a_squared * (c.x - b.x) + b_squared * (a.x - c.x) +
         c_squared * (b.x - a.x)) /
        denominator;
    output.radius_squared =
        (output.center.x - a.x) * (output.center.x - a.x) +
        (output.center.y - a.y) * (output.center.y - a.y);
    return std::isfinite(output.radius_squared);
}

bool pointInsideCircumcircle(const Point2D& point, const Triangle& triangle)
{
    const double distance_squared =
        (point.x - triangle.center.x) * (point.x - triangle.center.x) +
        (point.y - triangle.center.y) * (point.y - triangle.center.y);
    const double tolerance = 1.0e-9 * std::max(1.0, triangle.radius_squared);
    return distance_squared <= triangle.radius_squared + tolerance;
}

std::set<EdgeKey> buildDelaunayEdges(const std::vector<ConePoint>& cones)
{
    std::set<EdgeKey> edges;
    if (cones.size() < 3U)
    {
        return edges;
    }

    std::vector<Point2D> points;
    points.reserve(cones.size() + 3U);
    double min_x = cones.front().position.x;
    double max_x = min_x;
    double min_y = cones.front().position.y;
    double max_y = min_y;
    for (const ConePoint& cone : cones)
    {
        points.push_back(cone.position);
        min_x = std::min(min_x, cone.position.x);
        max_x = std::max(max_x, cone.position.x);
        min_y = std::min(min_y, cone.position.y);
        max_y = std::max(max_y, cone.position.y);
    }

    const double middle_x = 0.5 * (min_x + max_x);
    const double middle_y = 0.5 * (min_y + max_y);
    const double span = std::max(1.0, std::max(max_x - min_x, max_y - min_y));
    const std::size_t super_first = points.size();
    points.push_back(Point2D{middle_x - 20.0 * span, middle_y - span});
    const std::size_t super_second = points.size();
    points.push_back(Point2D{middle_x, middle_y + 20.0 * span});
    const std::size_t super_third = points.size();
    points.push_back(Point2D{middle_x + 20.0 * span, middle_y - span});

    Triangle super_triangle;
    if (!makeTriangle(
            points, super_first, super_second, super_third, super_triangle))
    {
        return edges;
    }

    std::vector<Triangle> triangles(1U, super_triangle);
    for (std::size_t point_index = 0U; point_index < cones.size(); ++point_index)
    {
        std::map<EdgeKey, int> boundary_counts;
        std::vector<Triangle> remaining;
        remaining.reserve(triangles.size());

        for (const Triangle& triangle : triangles)
        {
            if (!pointInsideCircumcircle(points[point_index], triangle))
            {
                remaining.push_back(triangle);
                continue;
            }
            ++boundary_counts[EdgeKey(triangle.first, triangle.second)];
            ++boundary_counts[EdgeKey(triangle.second, triangle.third)];
            ++boundary_counts[EdgeKey(triangle.third, triangle.first)];
        }

        for (const auto& boundary : boundary_counts)
        {
            if (boundary.second != 1)
            {
                continue;
            }
            Triangle triangle;
            if (makeTriangle(
                    points,
                    boundary.first.first,
                    boundary.first.second,
                    point_index,
                    triangle))
            {
                remaining.push_back(triangle);
            }
        }
        triangles.swap(remaining);
    }

    for (const Triangle& triangle : triangles)
    {
        if (triangle.first >= cones.size() || triangle.second >= cones.size() ||
            triangle.third >= cones.size())
        {
            continue;
        }
        edges.insert(EdgeKey(triangle.first, triangle.second));
        edges.insert(EdgeKey(triangle.second, triangle.third));
        edges.insert(EdgeKey(triangle.third, triangle.first));
    }
    return edges;
}

std::set<EdgeKey> buildPairwiseEdges(std::size_t cone_count)
{
    std::set<EdgeKey> edges;
    for (std::size_t first = 0U; first < cone_count; ++first)
    {
        for (std::size_t second = first + 1U; second < cone_count; ++second)
        {
            edges.insert(EdgeKey(first, second));
        }
    }
    return edges;
}

ReferenceInfo findReferenceInfo(
    const Point2D& point,
    const std::vector<Point2D>& reference_path)
{
    ReferenceInfo result;
    if (reference_path.size() < 2U)
    {
        return result;
    }

    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index + 1U < reference_path.size(); ++index)
    {
        const double dx = reference_path[index + 1U].x - reference_path[index].x;
        const double dy = reference_path[index + 1U].y - reference_path[index].y;
        const double length_squared = dx * dx + dy * dy;
        if (length_squared < 1.0e-10)
        {
            continue;
        }

        const double projection = std::max(0.0, std::min(1.0,
            ((point.x - reference_path[index].x) * dx +
             (point.y - reference_path[index].y) * dy) /
            length_squared));
        const double nearest_x = reference_path[index].x + projection * dx;
        const double nearest_y = reference_path[index].y + projection * dy;
        const double distance = std::hypot(point.x - nearest_x, point.y - nearest_y);
        if (distance < best_distance)
        {
            const double length = std::sqrt(length_squared);
            best_distance = distance;
            result.valid = true;
            result.tangent.x = dx / length;
            result.tangent.y = dy / length;
            result.distance = distance;
        }
    }
    return result;
}

}  // namespace

CandidateEdgeGenerator::CandidateEdgeGenerator(
    const CandidateEdgeGeneratorConfig& config)
    : config_(config)
{
}

std::vector<CandidateEdge> CandidateEdgeGenerator::generate(
    const std::vector<ConePoint>& cones,
    const std::vector<Point2D>& reference_path) const
{
    std::vector<CandidateEdge> candidates;
    if (cones.size() < 2U || config_.min_width <= 0.0 ||
        config_.max_width <= config_.min_width || config_.expected_width <= 0.0 ||
        config_.max_longitudinal_offset <= 0.0)
    {
        return candidates;
    }

    const auto filter_edges =
        [&](const std::set<EdgeKey>& raw_edges)
        {
            std::vector<CandidateEdge> filtered;
            for (const EdgeKey& key : raw_edges)
            {
                const ConePoint& first = cones[key.first];
                const ConePoint& second = cones[key.second];
                const double dx = second.position.x - first.position.x;
                const double dy = second.position.y - first.position.y;
                const double width = std::hypot(dx, dy);
                if (!std::isfinite(width) || width < config_.min_width ||
                    width > config_.max_width)
                {
                    continue;
                }

                Point2D midpoint;
                midpoint.x = 0.5 * (first.position.x + second.position.x);
                midpoint.y = 0.5 * (first.position.y + second.position.y);
                if (midpoint.x < config_.min_midpoint_x)
                {
                    continue;
                }

                const ReferenceInfo reference =
                    findReferenceInfo(midpoint, reference_path);
                if (reference.valid &&
                    reference.distance > config_.max_reference_distance)
                {
                    continue;
                }
                const double tangent_x = reference.valid ? reference.tangent.x : 1.0;
                const double tangent_y = reference.valid ? reference.tangent.y : 0.0;
                const double longitudinal_offset =
                    std::abs(dx * tangent_x + dy * tangent_y);
                if (longitudinal_offset > config_.max_longitudinal_offset)
                {
                    continue;
                }

                const double confidence =
                    std::min(first.confidence, second.confidence);
                const double width_error =
                    std::abs(width - config_.expected_width) /
                    config_.expected_width;
                const double longitudinal_error =
                    longitudinal_offset / config_.max_longitudinal_offset;
                const double reference_error = reference.valid ?
                    reference.distance / std::max(0.1, config_.expected_width) : 0.0;

                CandidateEdge edge;
                edge.first_cone_index = key.first;
                edge.second_cone_index = key.second;
                edge.first_cone_id = first.id;
                edge.second_cone_id = second.id;
                edge.midpoint = midpoint;
                edge.width = width;
                edge.confidence = confidence;
                edge.cost = config_.width_cost_weight * width_error +
                    config_.longitudinal_cost_weight * longitudinal_error +
                    config_.confidence_cost_weight * (1.0 - confidence) +
                    config_.reference_cost_weight * reference_error;
                filtered.push_back(edge);
            }
            return filtered;
        };

    if (config_.use_delaunay)
    {
        candidates = filter_edges(buildDelaunayEdges(cones));
    }
    if ((!config_.use_delaunay ||
         candidates.size() < static_cast<std::size_t>(
             std::max(0, config_.min_delaunay_candidates))) &&
        config_.fallback_to_pairwise)
    {
        candidates = filter_edges(buildPairwiseEdges(cones.size()));
    }

    std::sort(
        candidates.begin(), candidates.end(),
        [](const CandidateEdge& left, const CandidateEdge& right)
        {
            return left.cost < right.cost;
        });

    if (config_.max_edges_per_cone <= 0)
    {
        return candidates;
    }

    // 每个锥桶只保留少量优质连接，搜索会稳定很多。
    std::vector<int> edge_counts(cones.size(), 0);
    std::vector<CandidateEdge> limited;
    limited.reserve(candidates.size());
    for (const CandidateEdge& edge : candidates)
    {
        if (edge_counts[edge.first_cone_index] >= config_.max_edges_per_cone ||
            edge_counts[edge.second_cone_index] >= config_.max_edges_per_cone)
        {
            continue;
        }
        limited.push_back(edge);
        ++edge_counts[edge.first_cone_index];
        ++edge_counts[edge.second_cone_index];
    }
    return limited;
}

}  // namespace local_path_planner
