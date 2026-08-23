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

std::vector<double> naturalSecondDerivatives(
    const std::vector<double>& knots,
    const std::vector<Point2D>& points,
    bool use_x)
{
    const std::size_t count = points.size();
    std::vector<double> lower(count, 0.0);
    std::vector<double> diagonal(count, 1.0);
    std::vector<double> upper(count, 0.0);
    std::vector<double> right_hand_side(count, 0.0);
    std::vector<double> result(count, 0.0);
    if (count < 3U || knots.size() != count)
    {
        return result;
    }

    const auto coordinate = [&](std::size_t index)
    {
        return use_x ? points[index].x : points[index].y;
    };
    for (std::size_t index = 1U; index + 1U < count; ++index)
    {
        const double previous_step = knots[index] - knots[index - 1U];
        const double next_step = knots[index + 1U] - knots[index];
        if (previous_step < 1.0e-6 || next_step < 1.0e-6)
        {
            continue;
        }
        lower[index] = previous_step;
        diagonal[index] = 2.0 * (previous_step + next_step);
        upper[index] = next_step;
        right_hand_side[index] = 6.0 * (
            (coordinate(index + 1U) - coordinate(index)) / next_step -
            (coordinate(index) - coordinate(index - 1U)) / previous_step);
    }

    for (std::size_t index = 1U; index < count; ++index)
    {
        const double divisor = std::max(1.0e-9, diagonal[index - 1U]);
        const double factor = lower[index] / divisor;
        diagonal[index] -= factor * upper[index - 1U];
        right_hand_side[index] -= factor * right_hand_side[index - 1U];
    }
    result.back() = right_hand_side.back() /
        std::max(1.0e-9, diagonal.back());
    for (std::size_t index = count - 1U; index-- > 0U;)
    {
        result[index] =
            (right_hand_side[index] - upper[index] * result[index + 1U]) /
            std::max(1.0e-9, diagonal[index]);
    }
    return result;
}

double naturalSplineCoordinate(
    double start_value,
    double end_value,
    double start_second_derivative,
    double end_second_derivative,
    double segment_length,
    double ratio)
{
    const double start_weight = 1.0 - ratio;
    const double end_weight = ratio;
    const double correction = segment_length * segment_length / 6.0;
    return start_weight * start_value + end_weight * end_value + correction * (
        (start_weight * start_weight * start_weight - start_weight) *
            start_second_derivative +
        (end_weight * end_weight * end_weight - end_weight) *
            end_second_derivative);
}

Point2D boundedNaturalSplinePoint(
    const Point2D& start,
    const Point2D& end,
    double start_second_x,
    double end_second_x,
    double start_second_y,
    double end_second_y,
    double segment_length,
    double ratio,
    double max_deviation)
{
    Point2D curved;
    curved.x = naturalSplineCoordinate(
        start.x, end.x, start_second_x, end_second_x, segment_length, ratio);
    curved.y = naturalSplineCoordinate(
        start.y, end.y, start_second_y, end_second_y, segment_length, ratio);

    const Point2D linear{
        start.x + ratio * (end.x - start.x),
        start.y + ratio * (end.y - start.y)};
    const double deviation = pointDistance(linear, curved);
    if (max_deviation > 0.0 && deviation > 1.0e-9)
    {
        // tanh is a smooth limiter.  A hard clip produces another curvature
        // corner exactly where the safety limit becomes active.
        const double limited_deviation = max_deviation * std::tanh(
            deviation / max_deviation);
        const double scale = limited_deviation / deviation;
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
        std::vector<double> knots(cleaned.size(), 0.0);
        for (std::size_t index = 1U; index < cleaned.size(); ++index)
        {
            // Chord-length parameterisation is less likely to overshoot when
            // cone-pair spacing is uneven than a simple point index.
            knots[index] = knots[index - 1U] +
                pointDistance(cleaned[index - 1U], cleaned[index]);
        }
        const std::vector<double> second_x = naturalSecondDerivatives(
            knots, cleaned, true);
        const std::vector<double> second_y = naturalSecondDerivatives(
            knots, cleaned, false);
        for (std::size_t segment = 0U; segment + 1U < cleaned.size(); ++segment)
        {
            const Point2D& start = cleaned[segment];
            const Point2D& end = cleaned[segment + 1U];
            const double segment_length = knots[segment + 1U] - knots[segment];
            const int samples = std::max(3, static_cast<int>(std::ceil(
                segment_length / config_.sample_spacing * 3.0)));
            for (int sample = 0; sample < samples; ++sample)
            {
                dense.push_back(boundedNaturalSplinePoint(
                    start,
                    end,
                    second_x[segment],
                    second_x[segment + 1U],
                    second_y[segment],
                    second_y[segment + 1U],
                    segment_length,
                    static_cast<double>(sample) / static_cast<double>(samples),
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
