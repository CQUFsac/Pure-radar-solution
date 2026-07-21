#include "lidar_cone_detector/adaptive_euclidean_clusterer.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include <pcl/common/point_tests.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace lidar_cone_detector
{

ClusterResult::ClusterResult() : cloud(new PointCloud)
{
}

AdaptiveEuclideanClusterer::AdaptiveEuclideanClusterer(
    const std::vector<RangeBandConfig>& range_bands,
    float sensor_origin_x,
    float sensor_origin_y)
    : sensor_origin_x_(sensor_origin_x),
      sensor_origin_y_(sensor_origin_y)
{
    setRangeBands(range_bands);
}

void AdaptiveEuclideanClusterer::setRangeBands(const std::vector<RangeBandConfig>& range_bands)
{
    validateRangeBands(range_bands);
    range_bands_ = range_bands;
}

const std::vector<RangeBandConfig>&
AdaptiveEuclideanClusterer::rangeBands() const
{
    return range_bands_;
}

void AdaptiveEuclideanClusterer::setSensorOrigin(float x, float y)
{
    sensor_origin_x_ = x;
    sensor_origin_y_ = y;
}

void AdaptiveEuclideanClusterer::validateRangeBands(const std::vector<RangeBandConfig>& range_bands)
{
    if (range_bands.empty())
    {
        throw std::invalid_argument("至少需要一个集群范围");
    }

    for (const auto& band : range_bands)
    {
        if (!(band.min_range >= 0.0F && band.max_range > band.min_range))
        {
            throw std::invalid_argument("无效的波段距离范围 " + band.name);
        }
        if (band.cluster_tolerance <= 0.0F)
        {
            throw std::invalid_argument(
                "对于频带，必须为正 " + band.name);
        }
        if (band.min_cluster_size <= 0 ||
            band.max_cluster_size < band.min_cluster_size)
        {
            throw std::invalid_argument(
                "对于频带，聚类大小限制无效 " + band.name);
        }
    }
}

std::vector<ClusterResult> AdaptiveEuclideanClusterer::cluster(const PointCloud::ConstPtr& input) const
{
    std::vector<ClusterResult> results;
    if (!input || input->empty())
    {
        return results;
    }

    std::vector<PointCloud::Ptr> band_clouds;
    std::vector<std::vector<int>> band_original_indices(range_bands_.size());
    band_clouds.reserve(range_bands_.size());

    for (std::size_t band_index = 0; band_index < range_bands_.size(); ++band_index)
    {
        PointCloud::Ptr band_cloud(new PointCloud);
        band_cloud->header = input->header;
        band_cloud->reserve(input->size() / range_bands_.size() + 1U);
        band_clouds.push_back(band_cloud);
    }

    for (std::size_t point_index = 0; point_index < input->size(); ++point_index)
    {
        const PointT& point = input->points[point_index];
        if (!pcl::isFinite(point))
        {
            continue;
        }

        const float range = std::hypot(
            point.x - sensor_origin_x_,
            point.y - sensor_origin_y_);
        for (std::size_t band_index = 0; band_index < range_bands_.size(); ++band_index)
        {
            const RangeBandConfig& band = range_bands_[band_index];
            const bool is_last_band = band_index + 1U == range_bands_.size();
            const bool inside = range >= band.min_range &&
                (range < band.max_range || (is_last_band && range <= band.max_range));

            if (inside)
            {
                band_clouds[band_index]->push_back(point);
                band_original_indices[band_index].push_back(
                    static_cast<int>(point_index));
                break;
            }
        }
    }

    std::uint32_t next_cluster_id = 0U;

    for (std::size_t band_index = 0; band_index < range_bands_.size(); ++band_index)
    {
        PointCloud::Ptr& band_cloud = band_clouds[band_index];
        const RangeBandConfig& band = range_bands_[band_index];

        band_cloud->width = static_cast<std::uint32_t>(band_cloud->size());
        band_cloud->height = 1U;
        band_cloud->is_dense = true;

        if (band_cloud->size() < static_cast<std::size_t>(band.min_cluster_size))
        {
            continue;
        }

        pcl::search::KdTree<PointT>::Ptr search_tree(
            new pcl::search::KdTree<PointT>);
        search_tree->setInputCloud(band_cloud);

        pcl::EuclideanClusterExtraction<PointT> extractor;
        extractor.setClusterTolerance(band.cluster_tolerance);
        extractor.setMinClusterSize(band.min_cluster_size);
        extractor.setMaxClusterSize(band.max_cluster_size);
        extractor.setSearchMethod(search_tree);
        extractor.setInputCloud(band_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        extractor.extract(cluster_indices);

        for (const auto& point_indices : cluster_indices)
        {
            ClusterResult result;
            result.id = next_cluster_id++;
            result.band_index = band_index;
            result.band_name = band.name;
            result.cloud->header = input->header;
            result.cloud->reserve(point_indices.indices.size());
            result.original_indices.reserve(point_indices.indices.size());

            float range_sum = 0.0F;
            for (const int local_index : point_indices.indices)
            {
                if (local_index < 0 ||
                    static_cast<std::size_t>(local_index) >= band_cloud->size())
                {
                    continue;
                }

                const PointT& point = band_cloud->points[local_index];
                result.cloud->push_back(point);
                result.original_indices.push_back(
                    band_original_indices[band_index][local_index]);
                range_sum += std::hypot(
                    point.x - sensor_origin_x_,
                    point.y - sensor_origin_y_);
            }

            if (result.cloud->empty())
            {
                continue;
            }

            result.cloud->width = static_cast<std::uint32_t>(result.cloud->size());
            result.cloud->height = 1U;
            result.cloud->is_dense = true;
            result.mean_range = range_sum / static_cast<float>(result.cloud->size());
            results.push_back(std::move(result));
        }
    }

    return results;
}

}  
