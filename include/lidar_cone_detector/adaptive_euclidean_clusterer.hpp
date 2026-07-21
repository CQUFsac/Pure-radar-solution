#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>

#include "lidar_cone_detector/point_type.hpp"

namespace lidar_cone_detector
{

using PointT = PointXYZIRT;
using PointCloud = pcl::PointCloud<PointT>;

struct RangeBandConfig
{
    std::string name;
    float min_range = 0.0F;
    float max_range = 0.0F;
    float cluster_tolerance = 0.0F;
    int min_cluster_size = 1;
    int max_cluster_size = 1000;
};

struct ClusterResult
{
    ClusterResult();

    std::uint32_t id = 0U;
    std::size_t band_index = 0U;
    std::string band_name;
    float mean_range = 0.0F;
    PointCloud::Ptr cloud;

    // Maps every cluster point back to the flattened input cloud index.
    std::vector<int> original_indices;
};

class AdaptiveEuclideanClusterer
{
public:
    explicit AdaptiveEuclideanClusterer(
        const std::vector<RangeBandConfig>& range_bands,
        float sensor_origin_x = 0.0F,
        float sensor_origin_y = 0.0F);

    void setRangeBands(const std::vector<RangeBandConfig>& range_bands);
    const std::vector<RangeBandConfig>& rangeBands() const;
    void setSensorOrigin(float x, float y);
    std::vector<ClusterResult> cluster(const PointCloud::ConstPtr& input) const;

private:
    static void validateRangeBands(
        const std::vector<RangeBandConfig>& range_bands);

    std::vector<RangeBandConfig> range_bands_;
    float sensor_origin_x_ = 0.0F;
    float sensor_origin_y_ = 0.0F;
};

}  // namespace lidar_cone_detector
