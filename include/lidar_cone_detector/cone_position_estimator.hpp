#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "lidar_cone_detector/cone_cluster_classifier.hpp"

namespace lidar_cone_detector
{

struct ConePositionEstimatorConfig
{
    double lidar_origin_x = 0.0;
    double lidar_origin_y = 0.0;
    double nominal_cone_radius = 0.11;
    double minimum_cone_radius = 0.04;
    double maximum_cone_radius = 0.25;
    double measured_radius_weight = 0.70;
    std::size_t min_points_for_measured_radius = 4U;
    double near_surface_quantile = 0.15;

    bool use_fixed_ground_z = false;
    double fixed_ground_z = 0.0;
    double base_z_offset = 0.0;

    std::array<double, 4> radial_sigma_by_band{{0.06, 0.12, 0.25, 0.45}};
    std::array<double, 4> tangential_sigma_by_band{{0.04, 0.09, 0.18, 0.35}};
    std::array<double, 4> vertical_sigma_by_band{{0.06, 0.08, 0.12, 0.20}};
    std::array<std::size_t, 4> reference_points_by_band{{8U, 6U, 4U, 3U}};
    double maximum_covariance_scale = 3.0;
    double epsilon = 1e-6;
};

struct ConePositionEstimate
{
    bool valid = false;
    std::uint32_t cluster_id = 0U;
    std::size_t band_index = 0U;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double estimated_radius = 0.0;
    double confidence = 0.0;
    std::array<double, 9> covariance{{0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0}};
};

class ConePositionEstimator
{
public:
    ConePositionEstimator();
    explicit ConePositionEstimator(
        const ConePositionEstimatorConfig& config);

    ConePositionEstimate estimate(
        const ClusterResult& cluster,
        const ClusterFeatures& features,
        const ConeClassification& classification) const;

private:
    ConePositionEstimatorConfig config_;
};

}  // namespace lidar_cone_detector
