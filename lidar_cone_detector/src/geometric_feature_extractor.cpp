#include "lidar_cone_detector/geometric_feature_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <pcl/common/point_tests.h>

namespace lidar_cone_detector
{
namespace
{

struct SliceAccumulator
{
    explicit SliceAccumulator(std::size_t required_points)
        : required_points(required_points)
    {
    }

    void add(double tangential)
    {
        ++point_count;
        minimum = std::min(minimum, tangential);
        maximum = std::max(maximum, tangential);
    }

    bool valid() const
    {
        return point_count >= required_points;
    }

    double width() const
    {
        return valid() ? maximum - minimum : 0.0;
    }

    std::size_t required_points = 2U;
    std::size_t point_count = 0U;
    double minimum = std::numeric_limits<double>::max();
    double maximum = std::numeric_limits<double>::lowest();
};

}  // namespace

GeometricFeatureExtractor::GeometricFeatureExtractor()
    : config_()
{
}

GeometricFeatureExtractor::GeometricFeatureExtractor(
    const GeometricFeatureExtractorConfig& config)
    : config_(config)
{
    if (config_.min_points_for_pca < 3U)
    {
        throw std::invalid_argument("min_points_for_pca must be at least 3");
    }
    if (config_.min_points_per_slice < 2U)
    {
        throw std::invalid_argument("min_points_per_slice must be at least 2");
    }
    if (!(config_.bottom_slice_limit > 0.0 &&
          config_.middle_slice_limit > config_.bottom_slice_limit &&
          config_.middle_slice_limit < 1.0))
    {
        throw std::invalid_argument("Invalid vertical slice limits");
    }
}

ClusterFeatures GeometricFeatureExtractor::extract(
    const ClusterResult& cluster) const
{
    ClusterFeatures features;
    features.cluster_id = cluster.id;
    features.band_index = cluster.band_index;
    features.band_name = cluster.band_name;

    if (!cluster.cloud || cluster.cloud->empty())
    {
        return features;
    }

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double min_z = std::numeric_limits<double>::max();
    double min_range = std::numeric_limits<double>::max();
    double min_intensity = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    double max_z = std::numeric_limits<double>::lowest();
    double max_range = std::numeric_limits<double>::lowest();
    double max_intensity = std::numeric_limits<double>::lowest();

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double sum_range = 0.0;
    double sum_intensity = 0.0;
    double sum_intensity_squared = 0.0;
    std::size_t valid_point_count = 0U;
    std::set<std::uint16_t> unique_rings;
    std::uint16_t min_ring = std::numeric_limits<std::uint16_t>::max();
    std::uint16_t max_ring = std::numeric_limits<std::uint16_t>::lowest();

    for (const PointT& point : cluster.cloud->points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }

        const double x = point.x;
        const double y = point.y;
        const double z = point.z;
        const double intensity = point.intensity;
        const double dx = x - config_.lidar_origin_x;
        const double dy = y - config_.lidar_origin_y;
        const double range = std::hypot(dx, dy);

        ++valid_point_count;
        sum_x += x;
        sum_y += y;
        sum_z += z;
        sum_range += range;
        sum_intensity += intensity;
        sum_intensity_squared += intensity * intensity;

        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
        min_z = std::min(min_z, z);
        max_z = std::max(max_z, z);
        min_range = std::min(min_range, range);
        max_range = std::max(max_range, range);
        min_intensity = std::min(min_intensity, intensity);
        max_intensity = std::max(max_intensity, intensity);
        unique_rings.insert(point.ring);
        min_ring = std::min(min_ring, point.ring);
        max_ring = std::max(max_ring, point.ring);
    }

    if (valid_point_count == 0U)
    {
        return features;
    }

    const double count = static_cast<double>(valid_point_count);
    features.point_count = valid_point_count;
    features.min_x = min_x;
    features.max_x = max_x;
    features.min_y = min_y;
    features.max_y = max_y;
    features.min_z = min_z;
    features.max_z = max_z;
    features.min_range = min_range;
    features.max_range = max_range;
    features.mean_range = sum_range / count;
    features.centroid_x = sum_x / count;
    features.centroid_y = sum_y / count;
    features.centroid_z = sum_z / count;
    features.x_extent = max_x - min_x;
    features.y_extent = max_y - min_y;
    features.height = max_z - min_z;
    features.intensity_min = min_intensity;
    features.intensity_max = max_intensity;
    features.intensity_mean = sum_intensity / count;

    const double intensity_variance = std::max(
        0.0,
        sum_intensity_squared / count -
            features.intensity_mean * features.intensity_mean);
    features.intensity_stddev = std::sqrt(intensity_variance);
    features.unique_ring_count = unique_rings.size();
    features.min_ring = min_ring;
    features.max_ring = max_ring;
    features.ring_span = static_cast<std::uint16_t>(max_ring - min_ring);
    features.basic_valid = true;

    const double direction_x = features.centroid_x - config_.lidar_origin_x;
    const double direction_y = features.centroid_y - config_.lidar_origin_y;
    const double direction_length = std::hypot(direction_x, direction_y);
    if (direction_length <= config_.epsilon)
    {
        return features;
    }

    const double radial_x = direction_x / direction_length;
    const double radial_y = direction_y / direction_length;
    const double tangential_x = -radial_y;
    const double tangential_y = radial_x;
    double min_radial = std::numeric_limits<double>::max();
    double max_radial = std::numeric_limits<double>::lowest();
    double min_tangential = std::numeric_limits<double>::max();
    double max_tangential = std::numeric_limits<double>::lowest();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    SliceAccumulator bottom_slice(config_.min_points_per_slice);
    SliceAccumulator middle_slice(config_.min_points_per_slice);
    SliceAccumulator top_slice(config_.min_points_per_slice);

    for (const PointT& point : cluster.cloud->points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }

        const double dx = point.x - config_.lidar_origin_x;
        const double dy = point.y - config_.lidar_origin_y;
        const double radial = dx * radial_x + dy * radial_y;
        const double tangential = dx * tangential_x + dy * tangential_y;
        min_radial = std::min(min_radial, radial);
        max_radial = std::max(max_radial, radial);
        min_tangential = std::min(min_tangential, tangential);
        max_tangential = std::max(max_tangential, tangential);

        Eigen::Vector3d difference;
        difference << point.x - features.centroid_x,
            point.y - features.centroid_y,
            point.z - features.centroid_z;
        covariance += difference * difference.transpose();

        if (features.height > config_.epsilon)
        {
            const double normalized_height =
                (point.z - features.min_z) / features.height;
            if (normalized_height < config_.bottom_slice_limit)
            {
                bottom_slice.add(tangential);
            }
            else if (normalized_height < config_.middle_slice_limit)
            {
                middle_slice.add(tangential);
            }
            else
            {
                top_slice.add(tangential);
            }
        }
    }

    features.radial_depth = max_radial - min_radial;
    features.tangential_width = max_tangential - min_tangential;
    features.radial_valid = true;
    features.bottom_point_count = bottom_slice.point_count;
    features.middle_point_count = middle_slice.point_count;
    features.top_point_count = top_slice.point_count;
    features.bottom_width = bottom_slice.width();
    features.middle_width = middle_slice.width();
    features.top_width = top_slice.width();

    if (bottom_slice.valid() && top_slice.valid() &&
        features.bottom_width > config_.epsilon)
    {
        features.taper_ratio = features.top_width / features.bottom_width;
        features.slice_valid = true;
    }

    if (valid_point_count < config_.min_points_for_pca)
    {
        return features;
    }

    covariance /= count;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
    if (eigen_solver.info() != Eigen::Success)
    {
        return features;
    }

    const Eigen::Vector3d eigenvalues = eigen_solver.eigenvalues();
    features.lambda1 = std::max(0.0, eigenvalues(2));
    features.lambda2 = std::max(0.0, eigenvalues(1));
    features.lambda3 = std::max(0.0, eigenvalues(0));
    if (features.lambda1 <= config_.epsilon)
    {
        return features;
    }

    features.linearity =
        (features.lambda1 - features.lambda2) / features.lambda1;
    features.planarity =
        (features.lambda2 - features.lambda3) / features.lambda1;
    features.scattering = features.lambda3 / features.lambda1;
    const Eigen::Vector3d principal_direction =
        eigen_solver.eigenvectors().col(2);
    features.verticality = std::abs(principal_direction.z());
    features.pca_valid = true;
    return features;
}

}  // namespace lidar_cone_detector
