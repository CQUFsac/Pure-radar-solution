#include "local_path_planner/path_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace local_path_planner
{
namespace
{

double pointDistance(const Point2D& first, const Point2D& second)
{
    return std::hypot(second.x - first.x, second.y - first.y);
}

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

Point2D cubicPoint(
    const Point2D& first,
    const Point2D& start,
    const Point2D& end,
    const Point2D& last,
    double ratio,
    double tension,
    double max_deviation)
{
    const double tangent_scale = 0.5 * (1.0 -
        std::max(0.0, std::min(1.0, tension)));
    const Point2D start_tangent{
        tangent_scale * (end.x - first.x),
        tangent_scale * (end.y - first.y)};
    const Point2D end_tangent{
        tangent_scale * (last.x - start.x),
        tangent_scale * (last.y - start.y)};

    const double ratio_squared = ratio * ratio;
    const double ratio_cubed = ratio_squared * ratio;
    const double h00 = 2.0 * ratio_cubed - 3.0 * ratio_squared + 1.0;
    const double h10 = ratio_cubed - 2.0 * ratio_squared + ratio;
    const double h01 = -2.0 * ratio_cubed + 3.0 * ratio_squared;
    const double h11 = ratio_cubed - ratio_squared;

    Point2D curved;
    curved.x = h00 * start.x + h10 * start_tangent.x +
        h01 * end.x + h11 * end_tangent.x;
    curved.y = h00 * start.y + h10 * start_tangent.y +
        h01 * end.y + h11 * end_tangent.y;

    Point2D linear;
    linear.x = start.x + ratio * (end.x - start.x);
    linear.y = start.y + ratio * (end.y - start.y);
    const double deviation = pointDistance(linear, curved);
    if (max_deviation > 0.0 && deviation > max_deviation)
    {
        const double scale = max_deviation / deviation;
        curved.x = linear.x + scale * (curved.x - linear.x);
        curved.y = linear.y + scale * (curved.y - linear.y);
    }
    return curved;
}

std::vector<Point2D> resampleByDistance(
    const std::vector<Point2D>& input,
    double spacing)
{
    std::vector<Point2D> sampled;
    if (input.size() < 2U || spacing <= 0.0)
    {
        return sampled;
    }

    std::vector<double> cumulative(input.size(), 0.0);
    for (std::size_t index = 1U; index < input.size(); ++index)
    {
        cumulative[index] = cumulative[index - 1U] +
            pointDistance(input[index - 1U], input[index]);
    }
    const double total_length = cumulative.back();
    if (total_length < 1.0e-6)
    {
        return sampled;
    }

    std::size_t segment = 0U;
    for (double target = 0.0; target < total_length; target += spacing)
    {
        while (segment + 1U < cumulative.size() &&
               cumulative[segment + 1U] < target)
        {
            ++segment;
        }
        if (segment + 1U >= input.size())
        {
            break;
        }
        const double length = cumulative[segment + 1U] - cumulative[segment];
        const double ratio = length > 1.0e-9 ?
            (target - cumulative[segment]) / length : 0.0;
        sampled.push_back(Point2D{
            input[segment].x + ratio * (input[segment + 1U].x - input[segment].x),
            input[segment].y + ratio * (input[segment + 1U].y - input[segment].y)});
    }
    if (sampled.empty() || pointDistance(sampled.back(), input.back()) > 1.0e-6)
    {
        sampled.push_back(input.back());
    }
    return sampled;
}

}  // namespace

PathSmoother::PathSmoother(const PathSmootherConfig& config)
    : config_(config)
{
}

std::vector<PathPoint> PathSmoother::smooth(
    const std::vector<Point2D>& points,
    double confidence) const
{
    std::vector<PathPoint> output;
    if (points.size() < 2U || config_.sample_spacing <= 0.0)
    {
        return output;
    }

    std::vector<Point2D> cleaned;
    cleaned.push_back(points.front());
    for (std::size_t index = 1U; index < points.size(); ++index)
    {
        if (pointDistance(cleaned.back(), points[index]) >= config_.min_input_spacing)
        {
            cleaned.push_back(points[index]);
        }
    }
    if (cleaned.size() < 2U)
    {
        return output;
    }

    std::vector<Point2D> dense;
    if (config_.use_cubic_spline && cleaned.size() >= 3U)
    {
        for (std::size_t segment = 0U; segment + 1U < cleaned.size(); ++segment)
        {
            const Point2D& first = segment == 0U ?
                cleaned[segment] : cleaned[segment - 1U];
            const Point2D& start = cleaned[segment];
            const Point2D& end = cleaned[segment + 1U];
            const Point2D& last = segment + 2U < cleaned.size() ?
                cleaned[segment + 2U] : cleaned[segment + 1U];
            const int samples = std::max(3, static_cast<int>(std::ceil(
                pointDistance(start, end) / config_.sample_spacing * 3.0)));
            for (int sample = 0; sample < samples; ++sample)
            {
                dense.push_back(cubicPoint(
                    first,
                    start,
                    end,
                    last,
                    static_cast<double>(sample) / static_cast<double>(samples),
                    config_.spline_tension,
                    config_.max_spline_deviation));
            }
        }
        dense.push_back(cleaned.back());
    }
    else
    {
        dense = cleaned;
    }

    std::vector<Point2D> sampled =
        resampleByDistance(dense, config_.sample_spacing);
    for (int pass = 0; pass < std::max(0, config_.smoothing_passes); ++pass)
    {
        if (sampled.size() < 3U)
        {
            break;
        }
        std::vector<Point2D> next = sampled;
        for (std::size_t index = 1U; index + 1U < sampled.size(); ++index)
        {
            next[index].x = 0.2 * sampled[index - 1U].x +
                0.6 * sampled[index].x + 0.2 * sampled[index + 1U].x;
            next[index].y = 0.2 * sampled[index - 1U].y +
                0.6 * sampled[index].y + 0.2 * sampled[index + 1U].y;
        }
        sampled.swap(next);
    }

    confidence = std::max(0.0, std::min(1.0, confidence));
    output.resize(sampled.size());
    for (std::size_t index = 0U; index < sampled.size(); ++index)
    {
        const std::size_t previous = index == 0U ? 0U : index - 1U;
        const std::size_t next = std::min(index + 1U, sampled.size() - 1U);
        output[index].x = sampled[index].x;
        output[index].y = sampled[index].y;
        output[index].yaw = std::atan2(
            sampled[next].y - sampled[previous].y,
            sampled[next].x - sampled[previous].x);
        output[index].confidence = confidence;
    }

    for (std::size_t index = 1U; index + 1U < output.size(); ++index)
    {
        const double distance = std::hypot(
            output[index + 1U].x - output[index - 1U].x,
            output[index + 1U].y - output[index - 1U].y);
        if (distance > 1.0e-6)
        {
            output[index].curvature = normalizeAngle(
                output[index + 1U].yaw - output[index - 1U].yaw) / distance;
        }
    }
    if (output.size() > 2U)
    {
        output.front().curvature = output[1U].curvature;
        output.back().curvature = output[output.size() - 2U].curvature;
    }
    return output;
}

}  // namespace local_path_planner
