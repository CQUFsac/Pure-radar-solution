#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "lidar_cone_detector/adaptive_euclidean_clusterer.hpp"

namespace lidar_cone_detector
{

struct ClusterFeatures
{
    std::uint32_t cluster_id = 0U;
    std::size_t band_index = 0U;
    std::string band_name;

    bool basic_valid = false;
    bool radial_valid = false;
    bool pca_valid = false;
    bool slice_valid = false;

    std::size_t point_count = 0U;

    double min_range = 0.0;
    double max_range = 0.0;
    double mean_range = 0.0;

    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    double min_z = 0.0;
    double max_z = 0.0;

    double centroid_x = 0.0;
    double centroid_y = 0.0;
    double centroid_z = 0.0;

    double x_extent = 0.0;
    double y_extent = 0.0;
    double height = 0.0;
    double radial_depth = 0.0;
    double tangential_width = 0.0;

    double intensity_min = 0.0;
    double intensity_max = 0.0;
    double intensity_mean = 0.0;
    double intensity_stddev = 0.0;

    std::size_t unique_ring_count = 0U;
    std::uint16_t min_ring = 0U;
    std::uint16_t max_ring = 0U;
    std::uint16_t ring_span = 0U;

    double lambda1 = 0.0;
    double lambda2 = 0.0;
    double lambda3 = 0.0;
    double linearity = 0.0;
    double planarity = 0.0;
    double scattering = 0.0;
    double verticality = 0.0;

    std::size_t bottom_point_count = 0U;
    std::size_t middle_point_count = 0U;
    std::size_t top_point_count = 0U;
    double bottom_width = 0.0;
    double middle_width = 0.0;
    double top_width = 0.0;
    double taper_ratio = 0.0;
};

struct GeometricFeatureExtractorConfig
{
    double lidar_origin_x = 0.0;
    double lidar_origin_y = 0.0;
    std::size_t min_points_for_pca = 5U;
    std::size_t min_points_per_slice = 2U;
    double bottom_slice_limit = 0.35;
    double middle_slice_limit = 0.70;
    double epsilon = 1e-6;
};

class GeometricFeatureExtractor
{
public:
    GeometricFeatureExtractor();
    explicit GeometricFeatureExtractor(
        const GeometricFeatureExtractorConfig& config);

    ClusterFeatures extract(const ClusterResult& cluster) const;

private:
    GeometricFeatureExtractorConfig config_;
};

}  // namespace lidar_cone_detector
