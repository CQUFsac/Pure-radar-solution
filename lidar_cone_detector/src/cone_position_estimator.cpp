#include "lidar_cone_detector/cone_position_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <pcl/common/point_tests.h>

namespace lidar_cone_detector
{
namespace
{

double clamp(double value, double minimum, double maximum)
{
    return std::max(minimum, std::min(maximum, value));
}

std::size_t safeBandIndex(std::size_t band_index)
{
    return std::min<std::size_t>(band_index, 3U);
}

double quantile(std::vector<double> values, double probability)
{
    if (values.empty())
    {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const double position = clamp(probability, 0.0, 1.0) *
        static_cast<double>(values.size() - 1U);
    const std::size_t lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
    const double fraction = position - static_cast<double>(lower);
    return values[lower] * (1.0 - fraction) + values[upper] * fraction;
}

}  // namespace

ConePositionEstimator::ConePositionEstimator()
    : ConePositionEstimator(ConePositionEstimatorConfig())
{
}

ConePositionEstimator::ConePositionEstimator(
    const ConePositionEstimatorConfig& config)
    : config_(config)
{
    if (!(config_.minimum_cone_radius > 0.0 &&
          config_.nominal_cone_radius >= config_.minimum_cone_radius &&
          config_.maximum_cone_radius >= config_.nominal_cone_radius))
    {
        throw std::invalid_argument("Invalid cone radius configuration");
    }
    if (!(config_.near_surface_quantile >= 0.0 &&
          config_.near_surface_quantile <= 0.5))
    {
        throw std::invalid_argument("near_surface_quantile must be in [0, 0.5]");
    }
}

ConePositionEstimate ConePositionEstimator::estimate(
    const ClusterResult& cluster,
    const ClusterFeatures& features,
    const ConeClassification& classification) const
{
    ConePositionEstimate result;
    result.cluster_id = cluster.id;
    result.band_index = cluster.band_index;
    result.confidence = classification.confidence;

    if (!classification.is_candidate || !features.basic_valid ||
        !features.radial_valid || !cluster.cloud || cluster.cloud->empty())
    {
        return result;
    }

    const double direction_x = features.centroid_x - config_.lidar_origin_x;
    const double direction_y = features.centroid_y - config_.lidar_origin_y;
    const double direction_length = std::hypot(direction_x, direction_y);
    if (direction_length <= config_.epsilon)
    {
        return result;
    }

    const double radial_x = direction_x / direction_length;
    const double radial_y = direction_y / direction_length;
    const double tangential_x = -radial_y;
    const double tangential_y = radial_x;
    std::vector<double> radial_values;
    std::vector<double> tangential_values;
    radial_values.reserve(cluster.cloud->size());
    tangential_values.reserve(cluster.cloud->size());

    for (const PointT& point : cluster.cloud->points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }
        const double dx = point.x - config_.lidar_origin_x;
        const double dy = point.y - config_.lidar_origin_y;
        radial_values.push_back(dx * radial_x + dy * radial_y);
        tangential_values.push_back(dx * tangential_x + dy * tangential_y);
    }

    if (radial_values.empty())
    {
        return result;
    }

    const double near_surface = quantile(
        radial_values,
        config_.near_surface_quantile);
    const double tangential_center = quantile(tangential_values, 0.50);

    double radius = config_.nominal_cone_radius;
    if (features.point_count >= config_.min_points_for_measured_radius &&
        features.tangential_width > 2.0 * config_.minimum_cone_radius)
    {
        const double measured_radius = clamp(
            0.5 * features.tangential_width,
            config_.minimum_cone_radius,
            config_.maximum_cone_radius);
        const double weight = clamp(config_.measured_radius_weight, 0.0, 1.0);
        radius = weight * measured_radius +
            (1.0 - weight) * config_.nominal_cone_radius;
    }
    radius = clamp(
        radius,
        config_.minimum_cone_radius,
        config_.maximum_cone_radius);

    const double center_radial = near_surface + radius;
    result.x = config_.lidar_origin_x +
        center_radial * radial_x + tangential_center * tangential_x;
    result.y = config_.lidar_origin_y +
        center_radial * radial_y + tangential_center * tangential_y;
    result.z = (config_.use_fixed_ground_z ?
        config_.fixed_ground_z : features.min_z) + config_.base_z_offset;
    result.estimated_radius = radius;

    const std::size_t index = safeBandIndex(features.band_index);
    const double reference_points = static_cast<double>(
        std::max<std::size_t>(1U, config_.reference_points_by_band[index]));
    const double observed_points = static_cast<double>(
        std::max<std::size_t>(1U, features.point_count));
    const double point_scale = clamp(
        std::sqrt(reference_points / observed_points),
        0.70,
        config_.maximum_covariance_scale);
    const double confidence_scale = clamp(
        1.0 + 2.0 * (1.0 - classification.confidence),
        1.0,
        config_.maximum_covariance_scale);
    const double radial_sigma =
        config_.radial_sigma_by_band[index] * point_scale * confidence_scale;
    const double tangential_sigma =
        config_.tangential_sigma_by_band[index] * point_scale * confidence_scale;
    const double vertical_sigma =
        config_.vertical_sigma_by_band[index] * confidence_scale;
    const double radial_variance = radial_sigma * radial_sigma;
    const double tangential_variance = tangential_sigma * tangential_sigma;

    result.covariance[0] =
        radial_x * radial_x * radial_variance +
        tangential_x * tangential_x * tangential_variance;
    result.covariance[1] =
        radial_x * radial_y * radial_variance +
        tangential_x * tangential_y * tangential_variance;
    result.covariance[3] = result.covariance[1];
    result.covariance[4] =
        radial_y * radial_y * radial_variance +
        tangential_y * tangential_y * tangential_variance;
    result.covariance[8] = vertical_sigma * vertical_sigma;
    result.valid = std::isfinite(result.x) &&
        std::isfinite(result.y) && std::isfinite(result.z);
    return result;
}

}  // namespace lidar_cone_detector
