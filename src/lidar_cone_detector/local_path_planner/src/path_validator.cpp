#include "local_path_planner/path_validator.hpp"

#include <algorithm>
#include <cmath>

namespace local_path_planner
{
namespace
{

double normalizeAngle(double angle)
{
    const double pi = 3.14159265358979323846;
    while (angle > pi)
    {
        angle -= 2.0 * pi;
    }
    while (angle < -pi)
    {
        angle += 2.0 * pi;
    }
    return angle;
}

double cross(const PathPoint& a, const PathPoint& b, const PathPoint& c)
{
    return (b.x - a.x) * (c.y - a.y) -
        (b.y - a.y) * (c.x - a.x);
}

bool segmentsIntersect(
    const PathPoint& first_start,
    const PathPoint& first_end,
    const PathPoint& second_start,
    const PathPoint& second_end)
{
    return cross(first_start, first_end, second_start) *
               cross(first_start, first_end, second_end) < -1.0e-10 &&
        cross(second_start, second_end, first_start) *
               cross(second_start, second_end, first_end) < -1.0e-10;
}

}  // namespace

PathValidator::PathValidator(const PathValidatorConfig& config)
    : config_(config)
{
}

bool PathValidator::validate(
    const std::vector<PathPoint>& path,
    const std::vector<ConePoint>& cones,
    std::string& reason) const
{
    if (path.size() < static_cast<std::size_t>(std::max(1, config_.min_path_points)))
    {
        reason = "too few path points";
        return false;
    }
    if (path.front().confidence < config_.min_path_confidence)
    {
        reason = "path confidence is too low";
        return false;
    }

    double path_length = 0.0;
    double max_progress = 0.0;
    for (std::size_t index = 0U; index < path.size(); ++index)
    {
        if (!std::isfinite(path[index].x) || !std::isfinite(path[index].y) ||
            !std::isfinite(path[index].yaw) ||
            !std::isfinite(path[index].curvature))
        {
            reason = "path contains invalid numbers";
            return false;
        }
        if (std::abs(path[index].curvature) > config_.max_abs_curvature)
        {
            reason = "path curvature is too large";
            return false;
        }
        max_progress = std::max(max_progress, std::hypot(
            path[index].x - path.front().x,
            path[index].y - path.front().y));

        if (index > 0U)
        {
            const double spacing = std::hypot(
                path[index].x - path[index - 1U].x,
                path[index].y - path[index - 1U].y);
            if (spacing > config_.max_point_spacing)
            {
                reason = "path point gap is too large";
                return false;
            }
            if (std::abs(normalizeAngle(
                    path[index].yaw - path[index - 1U].yaw)) >
                config_.max_heading_change)
            {
                reason = "path heading changes too fast";
                return false;
            }
            if (std::abs(
                    path[index].curvature - path[index - 1U].curvature) >
                config_.max_curvature_change)
            {
                reason = "path curvature changes too fast";
                return false;
            }
            path_length += spacing;
        }
    }

    if (path_length < config_.min_path_length ||
        max_progress < config_.min_forward_progress)
    {
        reason = "path is too short";
        return false;
    }

    for (std::size_t first = 0U; first + 1U < path.size(); ++first)
    {
        for (std::size_t second = first + 2U; second + 1U < path.size(); ++second)
        {
            if (segmentsIntersect(
                    path[first], path[first + 1U],
                    path[second], path[second + 1U]))
            {
                reason = "path intersects itself";
                return false;
            }
        }
    }

    for (const PathPoint& point : path)
    {
        for (const ConePoint& cone : cones)
        {
            if (std::hypot(
                    point.x - cone.position.x,
                    point.y - cone.position.y) < config_.min_cone_clearance)
            {
                reason = "path is too close to a cone";
                return false;
            }
        }
    }

    reason = "ok";
    return true;
}

}  // namespace local_path_planner
