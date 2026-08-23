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
    double max_reach = 0.0;
    double cost = 0.0;
    double confidence_sum = 0.0;
    int gap_bridges = 0;
    bool reference_progress_valid = false;
    double reference_progress = 0.0;
    bool signed_turn_valid = false;
    double last_signed_turn = 0.0;
};

struct ReferenceProjection
{
    bool valid = false;
    double distance = 0.0;
    double progress = 0.0;
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

double crossTrackAlignmentError(
    const CandidateEdge& edge,
    const Point2D& path_direction)
{
    if (!edge.has_left_boundary || !edge.has_right_boundary)
    {
        return 0.0;
    }
    const double boundary_x =
        edge.right_boundary.x - edge.left_boundary.x;
    const double boundary_y =
        edge.right_boundary.y - edge.left_boundary.y;
    const double boundary_length = std::hypot(boundary_x, boundary_y);
    const double direction_length = std::hypot(
        path_direction.x, path_direction.y);
    if (boundary_length < 1.0e-6 || direction_length < 1.0e-6)
    {
        return 0.0;
    }
    // A true cross-track cone pair is approximately perpendicular to the
    // centreline direction.  Diagonal blue/yellow pairings leak a large
    // longitudinal component and pull the path towards the outside edge.
    return std::abs(
        (boundary_x * path_direction.x +
         boundary_y * path_direction.y) /
        (boundary_length * direction_length));
}

ReferenceProjection projectToPath(
    const Point2D& point,
    const std::vector<Point2D>& path)
{
    ReferenceProjection result;
    if (path.size() < 2U)
    {
        return result;
    }

    double best_distance = std::numeric_limits<double>::infinity();
    double accumulated_length = 0.0;
    for (std::size_t index = 0U; index + 1U < path.size(); ++index)
    {
        const double dx = path[index + 1U].x - path[index].x;
        const double dy = path[index + 1U].y - path[index].y;
        const double length_squared = dx * dx + dy * dy;
        if (length_squared < 1.0e-10)
        {
            continue;
        }
        const double raw_ratio =
            ((point.x - path[index].x) * dx +
             (point.y - path[index].y) * dy) /
            length_squared;
        double ratio = std::max(0.0, std::min(1.0, raw_ratio));
        if (index + 2U == path.size() && raw_ratio > 1.0)
        {
            // A point just beyond the old endpoint may continue along the
            // final tangent. The progress gate still limits the jump.
            ratio = raw_ratio;
        }
        const double nearest_x = path[index].x + ratio * dx;
        const double nearest_y = path[index].y + ratio * dy;
        const double distance = std::hypot(
            point.x - nearest_x, point.y - nearest_y);
        if (distance < best_distance)
        {
            best_distance = distance;
            result.valid = true;
            result.distance = distance;
            result.progress =
                accumulated_length + ratio * std::sqrt(length_squared);
        }
        accumulated_length += std::sqrt(length_squared);
    }
    return result;
}

double distanceToPath(
    const Point2D& point,
    const std::vector<Point2D>& path)
{
    const ReferenceProjection projection = projectToPath(point, path);
    return projection.valid ? projection.distance : 0.0;
}

double stateScore(
    const BeamState& state,
    const PathSearcherConfig& config)
{
    const double count = std::max<std::size_t>(1U, state.edge_indices.size());
    const double preferred_length = std::max(
        0.1, config.preferred_path_length);
    const double useful_length = std::min(state.max_reach, preferred_length);
    const double shortfall = std::max(
        0.0, preferred_length - state.max_reach) / preferred_length;
    return state.cost / static_cast<double>(count) -
        config.progress_reward_weight * useful_length +
        config.short_path_cost_weight * shortfall;
}

double missionTurnCost(
    const double signed_turn,
    const int desired_turn,
    const PathSearcherConfig& config)
{
    if (desired_turn == 0)
    {
        // Zero means that no direction hint is available.  Treating it as a
        // request to go straight makes every real corner unnecessarily
        // expensive and is the main cause of late turn-in.
        return 0.0;
    }

    const int turn_sign = signed_turn > config.mission_turn_deadband ? 1 :
        (signed_turn < -config.mission_turn_deadband ? -1 : 0);
    if (turn_sign == 0)
    {
        // When a mission explicitly requests a turn at the skidpad
        // crossover, a straight branch is not neutral: it delays turn-in and
        // can send the car through the wrong exit. The penalty is inactive in
        // normal trackdrive because desired_turn is zero there.
        return config.mission_straight_penalty;
    }
    if (turn_sign != 0 && turn_sign != desired_turn)
    {
        return config.mission_wrong_turn_penalty * std::abs(signed_turn);
    }
    return 0.0;
}

}  // namespace

PathSearcher::PathSearcher(const PathSearcherConfig& config)
    : config_(config)
{
}

SearchResult PathSearcher::search(
    const std::vector<CandidateEdge>& edges,
    const std::vector<Point2D>& reference_path,
    const int desired_turn) const
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
        const bool lateral_reference_start =
            config_.allow_reference_lateral_start &&
            reference_path.size() >= 2U &&
            edges[index].midpoint.x >= config_.min_start_forward_x;
        if (distance < config_.start_min_distance ||
            distance > config_.start_max_distance ||
            (edges[index].midpoint.x <= 0.0 && !lateral_reference_start))
        {
            continue;
        }

        const double heading = std::abs(std::atan2(
            edges[index].midpoint.y, edges[index].midpoint.x));
        if (heading > config_.max_start_heading)
        {
            continue;
        }
        const ReferenceProjection history_projection =
            projectToPath(edges[index].midpoint, reference_path);
        const double history_distance = history_projection.valid ?
            history_projection.distance : 0.0;
        if (config_.hard_reference_gate && reference_path.size() >= 2U &&
            history_distance > config_.max_reference_distance)
        {
            continue;
        }

        BeamState state;
        state.edge_indices.push_back(index);
        state.points.push_back(vehicle_origin);
        state.points.push_back(edges[index].midpoint);
        state.length = distance;
        state.max_reach = distance;
        state.confidence_sum = edges[index].confidence;
        state.direction.x = edges[index].midpoint.x / distance;
        state.direction.y = edges[index].midpoint.y / distance;
        state.reference_progress_valid = history_projection.valid;
        state.reference_progress = history_projection.progress;
        const double spacing_error = std::abs(
            distance - config_.expected_step_distance) /
            std::max(0.1, config_.expected_step_distance);
        state.cost = config_.edge_cost_weight * edges[index].cost +
            config_.heading_cost_weight * heading +
            config_.spacing_cost_weight * spacing_error +
            config_.history_cost_weight * history_distance +
            config_.cross_track_alignment_cost_weight *
                crossTrackAlignmentError(
                    edges[index], state.direction) +
            missionTurnCost(
                std::atan2(
                    edges[index].midpoint.y,
                    edges[index].midpoint.x),
                desired_turn,
                config_);
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
            return stateScore(left, config_) < stateScore(right, config_);
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
            stateScore(state, config_);
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
            const CandidateEdge& current_edge = edges[current_index];
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
                     config_.require_reference_for_gap_bridge &&
                     reference_path.size() < 2U) ||
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
                const double signed_turn = std::atan2(
                    state.direction.x * dy - state.direction.y * dx,
                    forward_projection);
                double turn_change_error = 0.0;
                double turn_reversal_error = 0.0;
                if (state.signed_turn_valid)
                {
                    turn_change_error = std::abs(
                        signed_turn - state.last_signed_turn) /
                        std::max(0.1, config_.max_turn_angle);
                    const double reversal_deadband = 0.04;
                    if (std::abs(signed_turn) > reversal_deadband &&
                        std::abs(state.last_signed_turn) > reversal_deadband &&
                        signed_turn * state.last_signed_turn < 0.0)
                    {
                        // A gentle S transition is necessary on consecutive
                        // bends. Penalise only how abrupt the sign reversal
                        // is, instead of assigning a fixed veto-like cost.
                        turn_reversal_error = std::min(
                            std::abs(signed_turn),
                            std::abs(state.last_signed_turn)) /
                            std::max(0.1, config_.max_turn_angle);
                    }
                }
                const double allowed_turn_angle = uses_gap_bridge ?
                    config_.max_gap_turn_angle : config_.max_turn_angle;
                if (turn_angle > allowed_turn_angle ||
                    wouldSelfIntersect(state.points, edges[index].midpoint))
                {
                    continue;
                }

                const ReferenceProjection history_projection =
                    projectToPath(edges[index].midpoint, reference_path);
                const double history_distance = history_projection.valid ?
                    history_projection.distance : 0.0;
                if (config_.hard_reference_gate &&
                    reference_path.size() >= 2U &&
                    history_distance > config_.max_reference_distance)
                {
                    continue;
                }
                double reference_progress_penalty = 0.0;
                if (state.reference_progress_valid &&
                    history_projection.valid)
                {
                    const double progress_step =
                        history_projection.progress -
                        state.reference_progress;
                    if (progress_step <
                            -config_.reference_backtrack_tolerance ||
                        progress_step >
                            config_.max_reference_progress_step)
                    {
                        if (config_.hard_reference_gate)
                        {
                            continue;
                        }
                        const double violation = progress_step <
                                -config_.reference_backtrack_tolerance ?
                            -config_.reference_backtrack_tolerance -
                                progress_step :
                            progress_step -
                                config_.max_reference_progress_step;
                        reference_progress_penalty = violation /
                            std::max(0.1, config_.expected_step_distance);
                    }
                }
                else if (uses_gap_bridge &&
                         config_.require_reference_for_gap_bridge)
                {
                    continue;
                }

                double reuse_penalty = 0.0;
                // Consecutive Delaunay cross-track edges normally share one
                // cone.  That is valid graph continuity, not repeated use.
                // Only penalise returning to cones used before the current
                // edge, which is what creates loops and zig-zag paths.
                for (std::size_t position = 0U;
                     position + 1U < state.edge_indices.size();
                     ++position)
                {
                    if (sharesCone(
                            edges[state.edge_indices[position]],
                            edges[index]))
                    {
                        reuse_penalty += config_.reused_cone_penalty;
                    }
                }
                const double disconnected_penalty =
                    sharesCone(current_edge, edges[index]) ? 0.0 :
                    std::max(0.0, config_.disconnected_edge_penalty);
                const double shared_transition_penalty =
                    sharesCone(current_edge, edges[index]) ?
                    std::max(
                        0.0,
                        config_.shared_cone_transition_penalty) : 0.0;
                const Point2D next_direction{
                    dx / distance, dy / distance};
                const double cross_track_alignment_error =
                    crossTrackAlignmentError(
                        edges[index], next_direction);
                const double spacing_error = std::abs(
                    distance - config_.expected_step_distance) /
                    std::max(0.1, config_.expected_step_distance);
                const double width_change_error = std::abs(
                    edges[index].width - current_edge.width) /
                    std::max(0.1, config_.expected_boundary_spacing);

                double boundary_spacing_error = 0.0;
                int boundary_advances = 0;
                const auto add_boundary_transition =
                    [&](const bool previous_valid,
                        const std::uint32_t previous_id,
                        const Point2D& previous_point,
                        const bool next_valid,
                        const std::uint32_t next_id,
                        const Point2D& next_point)
                    {
                        if (!previous_valid || !next_valid ||
                            previous_id == next_id)
                        {
                            return;
                        }
                        const double boundary_step = pointDistance(
                            previous_point, next_point);
                        const double expected_spacing =
                            std::max(
                                0.1,
                                config_.expected_boundary_spacing);
                        boundary_spacing_error += std::abs(
                            boundary_step -
                                config_.expected_boundary_spacing) /
                            expected_spacing;
                        if (boundary_step >
                            std::max(0.1, config_.max_boundary_step))
                        {
                            // Missing cones may create a long but still valid
                            // boundary step. Penalise it instead of deleting
                            // the only route through an acute bend.
                            boundary_spacing_error += 2.0 *
                                (boundary_step - config_.max_boundary_step) /
                                expected_spacing;
                        }
                        ++boundary_advances;
                    };
                add_boundary_transition(
                    current_edge.has_left_boundary,
                    current_edge.left_cone_id,
                    current_edge.left_boundary,
                    edges[index].has_left_boundary,
                    edges[index].left_cone_id,
                    edges[index].left_boundary);
                add_boundary_transition(
                    current_edge.has_right_boundary,
                    current_edge.right_cone_id,
                    current_edge.right_boundary,
                    edges[index].has_right_boundary,
                    edges[index].right_cone_id,
                    edges[index].right_boundary);
                if (boundary_advances > 0)
                {
                    boundary_spacing_error /=
                        static_cast<double>(boundary_advances);
                }

                BeamState next = state;
                next.edge_indices.push_back(index);
                next.points.push_back(edges[index].midpoint);
                next.length += distance;
                next.max_reach = std::max(
                    next.max_reach,
                    std::hypot(
                        edges[index].midpoint.x,
                        edges[index].midpoint.y));
                next.confidence_sum += edges[index].confidence;
                if (uses_gap_bridge)
                {
                    ++next.gap_bridges;
                }
                next.direction.x = dx / distance;
                next.direction.y = dy / distance;
                next.signed_turn_valid = true;
                next.last_signed_turn = signed_turn;
                next.reference_progress_valid =
                    history_projection.valid;
                next.reference_progress =
                    history_projection.progress;
                next.cost += config_.edge_cost_weight * edges[index].cost +
                    config_.heading_cost_weight * turn_angle +
                    config_.spacing_cost_weight * spacing_error +
                    config_.history_cost_weight * history_distance +
                    reuse_penalty +
                    config_.width_change_cost_weight *
                        width_change_error +
                    config_.boundary_spacing_cost_weight *
                        boundary_spacing_error +
                    config_.history_cost_weight *
                        reference_progress_penalty +
                    config_.turn_change_cost_weight * turn_change_error +
                    config_.turn_reversal_cost_weight *
                        turn_reversal_error +
                    config_.cross_track_alignment_cost_weight *
                        cross_track_alignment_error +
                    shared_transition_penalty +
                    disconnected_penalty +
                    missionTurnCost(
                        signed_turn, desired_turn, config_) +
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
                return stateScore(left, config_) < stateScore(right, config_);
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
