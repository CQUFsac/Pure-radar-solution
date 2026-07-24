#include "local_path_planner/path_searcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace local_path_planner
{
namespace
{

struct BeamState
{
    std::vector<std::size_t> edge_indices;
    std::vector<Point2D> points;
    Point2D direction{1.0, 0.0};
    double length = 0.0;
    double cost = 0.0;
    double confidence_sum = 0.0;
    int gap_bridges = 0;
};

double clampUnit(double value)
{
    return std::max(-1.0, std::min(1.0, value));
}

double pointDistance(const Point2D& first, const Point2D& second)
{
    return std::hypot(second.x - first.x, second.y - first.y);
}

double cross(const Point2D& a, const Point2D& b, const Point2D& c)
{
    return (b.x - a.x) * (c.y - a.y) -
        (b.y - a.y) * (c.x - a.x);
}

bool segmentsProperlyIntersect(
    const Point2D& first_start,
    const Point2D& first_end,
    const Point2D& second_start,
    const Point2D& second_end)
{
    const double first_side_a = cross(first_start, first_end, second_start);
    const double first_side_b = cross(first_start, first_end, second_end);
    const double second_side_a = cross(second_start, second_end, first_start);
    const double second_side_b = cross(second_start, second_end, first_end);
    return first_side_a * first_side_b < -1.0e-10 &&
        second_side_a * second_side_b < -1.0e-10;
}

bool wouldSelfIntersect(
    const std::vector<Point2D>& points,
    const Point2D& next_point)
{
    if (points.size() < 3U)
    {
        return false;
    }
    const Point2D& new_start = points.back();
    for (std::size_t index = 0U; index + 2U < points.size(); ++index)
    {
        if (segmentsProperlyIntersect(
                points[index], points[index + 1U], new_start, next_point))
        {
            return true;
        }
    }
    return false;
}

bool sharesCone(const CandidateEdge& first, const CandidateEdge& second)
{
    return first.first_cone_id == second.first_cone_id ||
        first.first_cone_id == second.second_cone_id ||
        first.second_cone_id == second.first_cone_id ||
        first.second_cone_id == second.second_cone_id;
}

double distanceToPath(
    const Point2D& point,
    const std::vector<Point2D>& path)
{
    if (path.size() < 2U)
    {
        return 0.0;
    }

    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index + 1U < path.size(); ++index)
    {
        const double dx = path[index + 1U].x - path[index].x;
        const double dy = path[index + 1U].y - path[index].y;
        const double length_squared = dx * dx + dy * dy;
        if (length_squared < 1.0e-10)
        {
            continue;
        }
        const double ratio = std::max(0.0, std::min(1.0,
            ((point.x - path[index].x) * dx +
             (point.y - path[index].y) * dy) /
            length_squared));
        const double nearest_x = path[index].x + ratio * dx;
        const double nearest_y = path[index].y + ratio * dy;
        best_distance = std::min(best_distance, std::hypot(
            point.x - nearest_x, point.y - nearest_y));
    }
    return std::isfinite(best_distance) ? best_distance : 0.0;
}

double stateScore(const BeamState& state, double progress_reward_weight)
{
    const double count = std::max<std::size_t>(1U, state.edge_indices.size());
    return state.cost / static_cast<double>(count) -
        progress_reward_weight * state.length;
}

}  // namespace

PathSearcher::PathSearcher(const PathSearcherConfig& config)
    : config_(config)
{
}

SearchResult PathSearcher::search(
    const std::vector<CandidateEdge>& edges,
    const std::vector<Point2D>& reference_path) const
{
    SearchResult result;
    if (edges.empty())
    {
        result.reason = "no candidate edges";
        return result;
    }

    const Point2D vehicle_origin;
    std::vector<BeamState> beam;
    beam.reserve(edges.size());
    for (std::size_t index = 0U; index < edges.size(); ++index)
    {
        const double distance = pointDistance(vehicle_origin, edges[index].midpoint);
        if (distance < config_.start_min_distance ||
            distance > config_.start_max_distance || edges[index].midpoint.x <= 0.0)
        {
            continue;
        }

        const double heading = std::abs(std::atan2(
            edges[index].midpoint.y, edges[index].midpoint.x));
        if (heading > config_.max_start_heading)
        {
            continue;
        }
        const double history_distance =
            distanceToPath(edges[index].midpoint, reference_path);
        if (reference_path.size() >= 2U &&
            history_distance > config_.max_reference_distance)
        {
            continue;
        }

        BeamState state;
        state.edge_indices.push_back(index);
        state.points.push_back(vehicle_origin);
        state.points.push_back(edges[index].midpoint);
        state.length = distance;
        state.confidence_sum = edges[index].confidence;
        state.direction.x = edges[index].midpoint.x / distance;
        state.direction.y = edges[index].midpoint.y / distance;
        const double spacing_error = std::abs(
            distance - config_.expected_step_distance) /
            std::max(0.1, config_.expected_step_distance);
        state.cost = config_.edge_cost_weight * edges[index].cost +
            config_.heading_cost_weight * heading +
            config_.spacing_cost_weight * spacing_error +
            config_.history_cost_weight * history_distance;
        beam.push_back(state);
    }

    if (beam.empty())
    {
        result.reason = "no valid start edge";
        return result;
    }

    std::sort(
        beam.begin(), beam.end(),
        [&](const BeamState& left, const BeamState& right)
        {
            return stateScore(left, config_.progress_reward_weight) <
                stateScore(right, config_.progress_reward_weight);
        });
    if (beam.size() > static_cast<std::size_t>(std::max(1, config_.beam_width)))
    {
        beam.resize(static_cast<std::size_t>(std::max(1, config_.beam_width)));
    }

    BeamState best_state;
    bool has_best_state = false;
    double best_score = std::numeric_limits<double>::infinity();

    const auto consider_state = [&](const BeamState& state,
                                    BeamState& best,
                                    bool& has_best,
                                    double& score)
    {
        if (state.edge_indices.size() < static_cast<std::size_t>(
                std::max(1, config_.min_midpoints)))
        {
            return;
        }
        const double candidate_score =
            stateScore(state, config_.progress_reward_weight);
        if (!has_best || candidate_score < score)
        {
            best = state;
            has_best = true;
            score = candidate_score;
        }
    };

    for (const BeamState& state : beam)
    {
        consider_state(state, best_state, has_best_state, best_score);
    }

    for (int depth = 1; depth < std::max(1, config_.max_midpoints); ++depth)
    {
        std::vector<BeamState> expanded;
        for (const BeamState& state : beam)
        {
            const std::size_t current_index = state.edge_indices.back();
            for (std::size_t index = 0U; index < edges.size(); ++index)
            {
                if (std::find(
                        state.edge_indices.begin(),
                        state.edge_indices.end(),
                        index) != state.edge_indices.end())
                {
                    continue;
                }

                const double dx = edges[index].midpoint.x - state.points.back().x;
                const double dy = edges[index].midpoint.y - state.points.back().y;
                const double distance = std::hypot(dx, dy);
                const bool uses_gap_bridge =
                    distance > config_.max_step_distance;
                if (distance < config_.min_step_distance ||
                    distance > config_.max_gap_step_distance ||
                    (uses_gap_bridge &&
                     state.gap_bridges >=
                         std::max(0, config_.max_gap_bridges)) ||
                    state.length + distance > config_.max_path_length)
                {
                    continue;
                }

                const double forward_projection =
                    dx * state.direction.x + dy * state.direction.y;
                if (forward_projection <= 0.0)
                {
                    continue;
                }
                const double turn_angle = std::acos(clampUnit(
                    forward_projection / std::max(1.0e-6, distance)));
                const double allowed_turn_angle = uses_gap_bridge ?
                    config_.max_gap_turn_angle : config_.max_turn_angle;
                if (turn_angle > allowed_turn_angle ||
                    wouldSelfIntersect(state.points, edges[index].midpoint))
                {
                    continue;
                }

                const double history_distance =
                    distanceToPath(edges[index].midpoint, reference_path);
                if (reference_path.size() >= 2U &&
                    history_distance > config_.max_reference_distance)
                {
                    continue;
                }

                double reuse_penalty = 0.0;
                for (std::size_t selected : state.edge_indices)
                {
                    if (sharesCone(edges[selected], edges[index]))
                    {
                        reuse_penalty += config_.reused_cone_penalty;
                    }
                }
                const double spacing_error = std::abs(
                    distance - config_.expected_step_distance) /
                    std::max(0.1, config_.expected_step_distance);

                BeamState next = state;
                next.edge_indices.push_back(index);
                next.points.push_back(edges[index].midpoint);
                next.length += distance;
                next.confidence_sum += edges[index].confidence;
                if (uses_gap_bridge)
                {
                    ++next.gap_bridges;
                }
                next.direction.x = dx / distance;
                next.direction.y = dy / distance;
                next.cost += config_.edge_cost_weight * edges[index].cost +
                    config_.heading_cost_weight * turn_angle +
                    config_.spacing_cost_weight * spacing_error +
                    config_.history_cost_weight * history_distance +
                    reuse_penalty +
                    (uses_gap_bridge ? config_.gap_bridge_penalty : 0.0);
                expanded.push_back(next);
            }
        }

        if (expanded.empty())
        {
            break;
        }
        std::sort(
            expanded.begin(), expanded.end(),
            [&](const BeamState& left, const BeamState& right)
            {
                return stateScore(left, config_.progress_reward_weight) <
                    stateScore(right, config_.progress_reward_weight);
            });
        if (expanded.size() > static_cast<std::size_t>(
                std::max(1, config_.beam_width)))
        {
            expanded.resize(static_cast<std::size_t>(
                std::max(1, config_.beam_width)));
        }
        beam.swap(expanded);
        for (const BeamState& state : beam)
        {
            consider_state(state, best_state, has_best_state, best_score);
        }
    }

    if (!has_best_state)
    {
        result.reason = "not enough connected midpoints";
        return result;
    }

    result.success = true;
    result.reason = "ok";
    result.selected_edge_indices = best_state.edge_indices;
    result.midpoints = best_state.points;
    result.gap_bridges = best_state.gap_bridges;
    result.cost = best_state.cost;
    result.confidence = best_state.confidence_sum /
        static_cast<double>(best_state.edge_indices.size());
    if (best_state.gap_bridges > 0)
    {
        result.confidence *= std::pow(
            std::max(0.0, std::min(1.0, config_.gap_confidence_scale)),
            best_state.gap_bridges);
    }
    return result;
}

}  // namespace local_path_planner
