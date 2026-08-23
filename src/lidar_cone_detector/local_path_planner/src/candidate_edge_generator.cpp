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
    double signed_offset = 0.0;
    double progress = 0.0;
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
    double accumulated_length = 0.0;
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
            result.signed_offset =
                -(point.x - nearest_x) * result.tangent.y +
                (point.y - nearest_y) * result.tangent.x;
            result.progress =
                accumulated_length + projection * length;
        }
        accumulated_length += std::sqrt(length_squared);
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

    const auto assign_boundary_info = [](
        CandidateEdge& edge,
        const ConePoint& first,
        const ConePoint& second,
        const bool same_semantic_boundary)
    {
        if (same_semantic_boundary)
        {
            const Point2D boundary_midpoint{
                0.5 * (first.position.x + second.position.x),
                0.5 * (first.position.y + second.position.y)};
            if (first.semantic_class == SEMANTIC_BLUE)
            {
                edge.has_left_boundary = true;
                edge.left_cone_id = std::min(first.id, second.id);
                edge.left_boundary = boundary_midpoint;
            }
            else if (first.semantic_class == SEMANTIC_YELLOW)
            {
                edge.has_right_boundary = true;
                edge.right_cone_id = std::min(first.id, second.id);
                edge.right_boundary = boundary_midpoint;
            }
            return;
        }

        const auto assign_known = [&](const ConePoint& cone)
        {
            if (cone.semantic_class == SEMANTIC_BLUE)
            {
                edge.has_left_boundary = true;
                edge.left_cone_id = cone.id;
                edge.left_boundary = cone.position;
            }
            else if (cone.semantic_class == SEMANTIC_YELLOW)
            {
                edge.has_right_boundary = true;
                edge.right_cone_id = cone.id;
                edge.right_boundary = cone.position;
            }
        };
        assign_known(first);
        assign_known(second);

        // Pure-LiDAR/unknown fallback: y-positive is provisionally left in
        // base_link. Semantic observations overwrite this approximation.
        if (!edge.has_left_boundary && !edge.has_right_boundary)
        {
            const ConePoint& left =
                first.position.y >= second.position.y ? first : second;
            const ConePoint& right =
                first.position.y >= second.position.y ? second : first;
            edge.has_left_boundary = true;
            edge.has_right_boundary = true;
            edge.left_cone_id = left.id;
            edge.right_cone_id = right.id;
            edge.left_boundary = left.position;
            edge.right_boundary = right.position;
        }
    };

    const auto filter_edges =
        [&](const std::set<EdgeKey>& raw_edges)
        {
            std::vector<CandidateEdge> filtered;
            for (const EdgeKey& key : raw_edges)
            {
                const ConePoint& first = cones[key.first];
                const ConePoint& second = cones[key.second];
                double semantic_penalty = 0.0;
                bool same_semantic_boundary = false;
                bool opposite_semantic_boundary = false;
                bool orange_cross_track_pair = false;
                if (config_.use_semantic_pairing)
                {
                    const bool first_is_boundary =
                        first.semantic_class == SEMANTIC_BLUE ||
                        first.semantic_class == SEMANTIC_YELLOW;
                    const bool second_is_boundary =
                        second.semantic_class == SEMANTIC_BLUE ||
                        second.semantic_class == SEMANTIC_YELLOW;
                    const bool first_is_orange =
                        first.semantic_class == SEMANTIC_SMALL_ORANGE ||
                        first.semantic_class == SEMANTIC_BIG_ORANGE;
                    const bool second_is_orange =
                        second.semantic_class == SEMANTIC_SMALL_ORANGE ||
                        second.semantic_class == SEMANTIC_BIG_ORANGE;

                    same_semantic_boundary =
                        first_is_boundary && second_is_boundary &&
                        first.semantic_class == second.semantic_class;
                    opposite_semantic_boundary =
                        first_is_boundary && second_is_boundary &&
                        first.semantic_class != second.semantic_class;
                    orange_cross_track_pair =
                        config_.allow_orange_cross_edges &&
                        first_is_orange && second_is_orange &&
                        first.position.y * second.position.y < 0.0;
                    if ((first_is_orange || second_is_orange) &&
                        !orange_cross_track_pair)
                    {
                        continue;
                    }
                    if (same_semantic_boundary &&
                        !config_.use_semantic_boundary_recovery)
                    {
                        continue;
                    }
                    if ((!first_is_boundary || !second_is_boundary) &&
                        !orange_cross_track_pair)
                    {
                        semantic_penalty =
                            std::max(0.0, config_.unknown_semantic_penalty);
                    }
                }
                const double dx = second.position.x - first.position.x;
                const double dy = second.position.y - first.position.y;
                const double width = std::hypot(dx, dy);
                if (same_semantic_boundary)
                {
                    if (!std::isfinite(width) ||
                        width < config_.min_boundary_link_distance ||
                        width > config_.max_boundary_link_distance)
                    {
                        continue;
                    }

                    Point2D boundary_midpoint;
                    boundary_midpoint.x =
                        0.5 * (first.position.x + second.position.x);
                    boundary_midpoint.y =
                        0.5 * (first.position.y + second.position.y);
                    const ReferenceInfo boundary_reference =
                        findReferenceInfo(boundary_midpoint, reference_path);
                    const ReferenceInfo first_reference =
                        findReferenceInfo(first.position, reference_path);
                    const ReferenceInfo second_reference =
                        findReferenceInfo(second.position, reference_path);
                    if (config_.semantic_boundary_requires_reference &&
                        !boundary_reference.valid)
                    {
                        continue;
                    }
                    if (config_.hard_reference_gate &&
                        first_reference.valid && second_reference.valid &&
                        std::abs(
                            first_reference.progress -
                            second_reference.progress) >
                            config_.max_boundary_reference_progress_step)
                    {
                        continue;
                    }

                    double tangent_x = dx / width;
                    double tangent_y = dy / width;
                    if (boundary_reference.valid)
                    {
                        const double alignment =
                            tangent_x * boundary_reference.tangent.x +
                            tangent_y * boundary_reference.tangent.y;
                        if (alignment < 0.0)
                        {
                            tangent_x = -tangent_x;
                            tangent_y = -tangent_y;
                        }
                        if (config_.hard_reference_gate)
                        {
                            const double minimum_alignment = std::cos(
                                std::max(
                                    0.0,
                                    config_.max_boundary_heading_error));
                            const double first_alignment = std::abs(
                                tangent_x * first_reference.tangent.x +
                                tangent_y * first_reference.tangent.y);
                            const double second_alignment = std::abs(
                                tangent_x * second_reference.tangent.x +
                                tangent_y * second_reference.tangent.y);
                            if (std::abs(alignment) < minimum_alignment ||
                                !first_reference.valid ||
                                !second_reference.valid ||
                                first_alignment < minimum_alignment ||
                                second_alignment < minimum_alignment)
                            {
                                continue;
                            }

                            const auto offset_is_valid =
                                [&](const ReferenceInfo& sample)
                                {
                                    const double absolute_offset =
                                        std::abs(sample.signed_offset);
                                    const bool correct_side =
                                        first.semantic_class == SEMANTIC_BLUE ?
                                        sample.signed_offset > 0.0 :
                                        sample.signed_offset < 0.0;
                                    return sample.valid && correct_side &&
                                        absolute_offset >=
                                            config_.min_boundary_reference_offset &&
                                        absolute_offset <=
                                            config_.max_boundary_reference_offset;
                                };
                            if (!offset_is_valid(first_reference) ||
                                !offset_is_valid(boundary_reference) ||
                                !offset_is_valid(second_reference))
                            {
                                continue;
                            }

                            // A boundary link must stay on the same side of
                            // the old centreline, not cut across a hairpin.
                            bool interior_samples_valid = true;
                            const double sample_ratios[] = {0.25, 0.50, 0.75};
                            for (const double ratio : sample_ratios)
                            {
                                Point2D sample_point;
                                sample_point.x =
                                    first.position.x + ratio * dx;
                                sample_point.y =
                                    first.position.y + ratio * dy;
                                if (!offset_is_valid(findReferenceInfo(
                                        sample_point, reference_path)))
                                {
                                    interior_samples_valid = false;
                                    break;
                                }
                            }
                            if (!interior_samples_valid)
                            {
                                continue;
                            }
                        }
                    }
                    else
                    {
                        // At startup there is no history, so only accept a
                        // same-colour link that generally continues forward.
                        if (tangent_x < 0.0)
                        {
                            tangent_x = -tangent_x;
                            tangent_y = -tangent_y;
                        }
                        const double minimum_forward_alignment = std::cos(
                            std::max(
                                0.0,
                                config_.max_boundary_start_heading_error));
                        if (tangent_x < minimum_forward_alignment)
                        {
                            continue;
                        }
                    }

                    const double half_width =
                        0.5 * config_.expected_width;
                    const double left_normal_x = -tangent_y;
                    const double left_normal_y = tangent_x;
                    Point2D virtual_center = boundary_midpoint;
                    if (first.semantic_class == SEMANTIC_BLUE)
                    {
                        virtual_center.x -= half_width * left_normal_x;
                        virtual_center.y -= half_width * left_normal_y;
                    }
                    else
                    {
                        virtual_center.x += half_width * left_normal_x;
                        virtual_center.y += half_width * left_normal_y;
                    }
                    if (virtual_center.x < config_.min_midpoint_x)
                    {
                        continue;
                    }

                    const ReferenceInfo center_reference =
                        findReferenceInfo(virtual_center, reference_path);
                    if (config_.hard_reference_gate &&
                        center_reference.valid &&
                        center_reference.distance >
                            config_.max_boundary_center_reference_distance)
                    {
                        continue;
                    }

                    const double confidence =
                        std::min(first.confidence, second.confidence);
                    const double spacing_error = std::abs(
                        width - config_.expected_boundary_spacing) /
                        std::max(0.1, config_.expected_boundary_spacing);
                    const double reference_error = center_reference.valid ?
                        center_reference.distance /
                            std::max(0.1, config_.expected_width) :
                        0.0;

                    CandidateEdge edge;
                    edge.first_cone_index = key.first;
                    edge.second_cone_index = key.second;
                    edge.first_cone_id = first.id;
                    edge.second_cone_id = second.id;
                    edge.midpoint = virtual_center;
                    edge.width = config_.expected_width;
                    edge.confidence = confidence * 0.80;
                    edge.virtual_from_boundary = true;
                    assign_boundary_info(edge, first, second, true);
                    edge.cost =
                        config_.width_cost_weight * spacing_error +
                        config_.confidence_cost_weight *
                            (1.0 - confidence) +
                        config_.reference_cost_weight * reference_error +
                        std::max(0.0, config_.boundary_virtual_cost);
                    filtered.push_back(edge);
                    continue;
                }

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
                if (config_.hard_reference_gate && reference.valid &&
                    reference.distance > config_.max_reference_distance)
                {
                    continue;
                }
                if (opposite_semantic_boundary)
                {
                    const ConePoint& blue =
                        first.semantic_class == SEMANTIC_BLUE ?
                        first : second;
                    const ConePoint& yellow =
                        first.semantic_class == SEMANTIC_YELLOW ?
                        first : second;
                    const ReferenceInfo blue_reference =
                        findReferenceInfo(blue.position, reference_path);
                    const ReferenceInfo yellow_reference =
                        findReferenceInfo(yellow.position, reference_path);
                    if (config_.hard_reference_gate &&
                        blue_reference.valid && yellow_reference.valid)
                    {
                        const double progress_difference = std::abs(
                            blue_reference.progress -
                            yellow_reference.progress);
                        const double maximum_progress_difference =
                            config_.max_pair_reference_progress_difference;
                        if (blue_reference.signed_offset <= 0.0 ||
                            yellow_reference.signed_offset >= 0.0 ||
                            progress_difference >
                                maximum_progress_difference)
                        {
                            continue;
                        }
                    }
                    else if (!reference.valid &&
                             blue.position.y <= yellow.position.y)
                    {
                        continue;
                    }
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
                assign_boundary_info(edge, first, second, false);
                edge.cost = config_.width_cost_weight * width_error +
                    config_.longitudinal_cost_weight * longitudinal_error +
                    config_.confidence_cost_weight * (1.0 - confidence) +
                    config_.reference_cost_weight * reference_error +
                    semantic_penalty;
                filtered.push_back(edge);
            }
            return filtered;
        };

    if (config_.use_delaunay)
    {
        std::set<EdgeKey> raw_edges = buildDelaunayEdges(cones);
        if (config_.use_semantic_pairing && config_.fallback_to_pairwise)
        {
            // Delaunay connectivity flips when four cones are almost
            // co-circular.  Keep Delaunay for topology, but always add every
            // plausible blue-yellow pair to the assignment candidate set so
            // the final one-to-one match does not depend on one diagonal.
            for (std::size_t first = 0U; first < cones.size(); ++first)
            {
                for (std::size_t second = first + 1U;
                     second < cones.size(); ++second)
                {
                    const bool opposite =
                        (cones[first].semantic_class == SEMANTIC_BLUE &&
                         cones[second].semantic_class == SEMANTIC_YELLOW) ||
                        (cones[first].semantic_class == SEMANTIC_YELLOW &&
                         cones[second].semantic_class == SEMANTIC_BLUE);
                    const bool first_is_orange =
                        cones[first].semantic_class == SEMANTIC_SMALL_ORANGE ||
                        cones[first].semantic_class == SEMANTIC_BIG_ORANGE;
                    const bool second_is_orange =
                        cones[second].semantic_class == SEMANTIC_SMALL_ORANGE ||
                        cones[second].semantic_class == SEMANTIC_BIG_ORANGE;
                    const bool orange_cross =
                        config_.allow_orange_cross_edges &&
                        first_is_orange && second_is_orange &&
                        cones[first].position.y * cones[second].position.y < 0.0;
                    if (opposite || orange_cross)
                    {
                        raw_edges.insert(EdgeKey(first, second));
                    }
                }
            }
        }
        candidates = filter_edges(raw_edges);
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

    if (config_.use_semantic_pairing &&
        config_.enforce_semantic_one_to_one)
    {
        // In semantic mode a physical blue/yellow cone may belong to only one
        // cross-track pair.  Keeping several low-cost alternatives for the
        // same cone makes the selected centreline switch every frame when the
        // Delaunay triangulation changes by one edge.
        std::set<std::size_t> matched_cross_track_cones;
        std::vector<CandidateEdge> one_to_one;
        one_to_one.reserve(candidates.size());
        for (const CandidateEdge& edge : candidates)
        {
            const ConePoint& first = cones[edge.first_cone_index];
            const ConePoint& second = cones[edge.second_cone_index];
            const bool is_opposite_semantic_pair =
                !edge.virtual_from_boundary &&
                ((first.semantic_class == SEMANTIC_BLUE &&
                  second.semantic_class == SEMANTIC_YELLOW) ||
                 (first.semantic_class == SEMANTIC_YELLOW &&
                  second.semantic_class == SEMANTIC_BLUE) ||
                 (config_.allow_orange_cross_edges &&
                  (first.semantic_class == SEMANTIC_SMALL_ORANGE ||
                   first.semantic_class == SEMANTIC_BIG_ORANGE) &&
                  (second.semantic_class == SEMANTIC_SMALL_ORANGE ||
                   second.semantic_class == SEMANTIC_BIG_ORANGE) &&
                  first.position.y * second.position.y < 0.0));
            if (is_opposite_semantic_pair)
            {
                if (matched_cross_track_cones.count(edge.first_cone_index) > 0U ||
                    matched_cross_track_cones.count(edge.second_cone_index) > 0U)
                {
                    continue;
                }
                matched_cross_track_cones.insert(edge.first_cone_index);
                matched_cross_track_cones.insert(edge.second_cone_index);
            }
            one_to_one.push_back(edge);
        }
        candidates.swap(one_to_one);
    }

    if (config_.max_edges_per_cone <= 0)
    {
        return candidates;
    }

    // 每个锥桶只保留少量优质连接，搜索会稳定很多。
    std::vector<int> edge_counts(cones.size(), 0);
    std::vector<int> boundary_edge_counts(cones.size(), 0);
    std::vector<CandidateEdge> limited;
    limited.reserve(candidates.size());
    for (const CandidateEdge& edge : candidates)
    {
        if (edge_counts[edge.first_cone_index] >= config_.max_edges_per_cone ||
            edge_counts[edge.second_cone_index] >= config_.max_edges_per_cone)
        {
            continue;
        }
        if (edge.virtual_from_boundary &&
            config_.max_boundary_edges_per_cone > 0 &&
            (boundary_edge_counts[edge.first_cone_index] >=
                 config_.max_boundary_edges_per_cone ||
             boundary_edge_counts[edge.second_cone_index] >=
                 config_.max_boundary_edges_per_cone))
        {
            continue;
        }
        limited.push_back(edge);
        ++edge_counts[edge.first_cone_index];
        ++edge_counts[edge.second_cone_index];
        if (edge.virtual_from_boundary)
        {
            ++boundary_edge_counts[edge.first_cone_index];
            ++boundary_edge_counts[edge.second_cone_index];
        }
    }
    return limited;
}

}  // namespace local_path_planner
