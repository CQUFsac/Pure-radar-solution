#include "local_path_planner/single_side_recovery.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace local_path_planner
{
namespace
{

double clamp(const double value, const double lower, const double upper)
{
    return std::max(lower, std::min(upper, value));
}

double distance(const Point2D& first, const Point2D& second)
{
    return std::hypot(first.x - second.x, first.y - second.y);
}

struct Projection
{
    bool valid = false;
    Point2D point;
    Point2D tangent;
    double signed_offset = 0.0;
    double progress = 0.0;
};

Projection projectToReference(
    const Point2D& input,
    const std::vector<Point2D>& reference)
{
    Projection best;
    double best_distance = std::numeric_limits<double>::infinity();
    double accumulated_length = 0.0;

    for (std::size_t index = 1U; index < reference.size(); ++index)
    {
        const Point2D& first = reference[index - 1U];
        const Point2D& second = reference[index];
        const double dx = second.x - first.x;
        const double dy = second.y - first.y;
        const double segment_length = std::hypot(dx, dy);
        if (segment_length < 1e-4)
        {
            continue;
        }

        const double tangent_x = dx / segment_length;
        const double tangent_y = dy / segment_length;
        const double raw_along =
            (input.x - first.x) * tangent_x +
            (input.y - first.y) * tangent_y;
        const double along = clamp(raw_along, 0.0, segment_length);
        Point2D projected;
        projected.x = first.x + tangent_x * along;
        projected.y = first.y + tangent_y * along;
        const double current_distance = distance(input, projected);

        if (current_distance < best_distance)
        {
            best_distance = current_distance;
            best.valid = true;
            best.point = projected;
            best.tangent.x = tangent_x;
            best.tangent.y = tangent_y;
            best.signed_offset =
                -(input.x - projected.x) * tangent_y +
                (input.y - projected.y) * tangent_x;
            best.progress = accumulated_length + along;

            // Allow cones just beyond the end of the old path to extend it.
            if (index + 1U == reference.size() && raw_along > segment_length)
            {
                best.point.x = first.x + tangent_x * raw_along;
                best.point.y = first.y + tangent_y * raw_along;
                best.signed_offset =
                    -(input.x - best.point.x) * tangent_y +
                    (input.y - best.point.y) * tangent_x;
                best.progress = accumulated_length + raw_along;
            }
        }
        accumulated_length += segment_length;
    }
    return best;
}

struct OrderedPoint
{
    Point2D point;
    double progress = 0.0;
    bool virtual_point = false;
    double confidence = 0.0;
};

}  // namespace

SingleSideRecovery::SingleSideRecovery(const SingleSideRecoveryConfig& config)
    : config_(config)
{
}

SingleSideRecoveryResult SingleSideRecovery::recover(
    const std::vector<ConePoint>& cones,
    const std::vector<Point2D>& reference_path,
    const double reference_confidence) const
{
    SingleSideRecoveryResult result;
    if (!config_.enabled)
    {
        result.reason = "single-side recovery disabled";
        return result;
    }
    if (reference_path.size() < 2U)
    {
        result.reason = "single-side recovery has no reference path";
        return result;
    }

    std::vector<OrderedPoint> ordered;
    ordered.reserve(reference_path.size() + cones.size());

    double reference_progress = 0.0;
    for (std::size_t index = 0U; index < reference_path.size(); ++index)
    {
        if (index > 0U)
        {
            reference_progress += distance(
                reference_path[index - 1U], reference_path[index]);
        }
        if (reference_path[index].x >= -0.20 &&
            reference_path[index].x <= config_.max_forward_x)
        {
            OrderedPoint point;
            point.point = reference_path[index];
            point.progress = reference_progress;
            point.confidence = reference_confidence;
            ordered.push_back(point);
        }
    }

    const double half_width =
        0.5 * std::max(0.5, config_.expected_track_width);
    double virtual_confidence_sum = 0.0;
    for (const ConePoint& cone : cones)
    {
        const Projection projection =
            projectToReference(cone.position, reference_path);
        if (!projection.valid)
        {
            continue;
        }

        const double absolute_offset = std::abs(projection.signed_offset);
        if (absolute_offset < config_.min_boundary_offset ||
            absolute_offset > config_.max_boundary_offset)
        {
            continue;
        }

        const double side = projection.signed_offset >= 0.0 ? 1.0 : -1.0;
        Point2D virtual_center;
        virtual_center.x =
            cone.position.x + side * half_width * projection.tangent.y;
        virtual_center.y =
            cone.position.y - side * half_width * projection.tangent.x;

        if (virtual_center.x < config_.min_forward_x ||
            virtual_center.x > config_.max_forward_x)
        {
            continue;
        }
        if (distance(virtual_center, projection.point) >
            config_.max_center_reference_distance)
        {
            continue;
        }

        OrderedPoint point;
        point.point = virtual_center;
        point.progress = projection.progress;
        point.virtual_point = true;
        point.confidence = cone.confidence;
        ordered.push_back(point);
        ++result.virtual_point_count;
        virtual_confidence_sum += cone.confidence;
    }

    if (result.virtual_point_count <
        std::max(1, config_.min_virtual_points))
    {
        result.reason = "no reliable unmatched boundary cone";
        return result;
    }

    std::stable_sort(
        ordered.begin(),
        ordered.end(),
        [](const OrderedPoint& first, const OrderedPoint& second)
        {
            if (std::abs(first.progress - second.progress) > 1e-5)
            {
                return first.progress < second.progress;
            }
            return !first.virtual_point && second.virtual_point;
        });

    // At almost the same progress, prefer the newly inferred point. It extends
    // the old path without creating a small zig-zag beside it.
    for (const OrderedPoint& candidate : ordered)
    {
        if (!result.centerline.empty() &&
            distance(result.centerline.back(), candidate.point) <
                config_.min_point_spacing)
        {
            if (candidate.virtual_point)
            {
                result.centerline.back() = candidate.point;
            }
            continue;
        }
        result.centerline.push_back(candidate.point);
    }

    if (result.centerline.size() < 2U)
    {
        result.centerline.clear();
        result.reason = "single-side centreline is too short";
        return result;
    }

    const double mean_virtual_confidence =
        virtual_confidence_sum /
        static_cast<double>(result.virtual_point_count);
    result.confidence = clamp(
        std::min(reference_confidence, mean_virtual_confidence) *
            config_.confidence_scale,
        0.0,
        1.0);
    result.success = true;
    result.reason = "ok";
    return result;
}

}  // namespace local_path_planner
