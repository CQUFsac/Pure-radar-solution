#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "driverless_msgs/ConeObservation.h"
#include "driverless_msgs/ConeObservationArray.h"
#include "lidar_cone_detector/adaptive_euclidean_clusterer.hpp"
#include "lidar_cone_detector/cone_cluster_classifier.hpp"
#include "lidar_cone_detector/cone_position_estimator.hpp"
#include "lidar_cone_detector/geometric_feature_extractor.hpp"

namespace lidar_cone_detector
{
namespace
{

const sensor_msgs::PointField* findField(
    const sensor_msgs::PointCloud2& message,
    const std::string& name)
{
    for (const auto& field : message.fields)
    {
        if (field.name == name)
        {
            return &field;
        }
    }
    return nullptr;
}

std::size_t dataTypeSize(std::uint8_t datatype)
{
    switch (datatype)
    {
        case sensor_msgs::PointField::INT8:
        case sensor_msgs::PointField::UINT8:
            return 1U;
        case sensor_msgs::PointField::INT16:
        case sensor_msgs::PointField::UINT16:
            return 2U;
        case sensor_msgs::PointField::INT32:
        case sensor_msgs::PointField::UINT32:
        case sensor_msgs::PointField::FLOAT32:
            return 4U;
        case sensor_msgs::PointField::FLOAT64:
            return 8U;
        default:
            return 0U;
    }
}

template <typename T>
T readUnaligned(const std::uint8_t* address)
{
    T value{};
    std::memcpy(&value, address, sizeof(T));
    return value;
}

bool readNumericField(
    const std::uint8_t* point_address,
    const sensor_msgs::PointField& field,
    double& output)
{
    switch (field.datatype)
    {
        case sensor_msgs::PointField::INT8:
            output = readUnaligned<std::int8_t>(point_address + field.offset);
            return true;
        case sensor_msgs::PointField::UINT8:
            output = readUnaligned<std::uint8_t>(point_address + field.offset);
            return true;
        case sensor_msgs::PointField::INT16:
            output = readUnaligned<std::int16_t>(point_address + field.offset);
            return true;
        case sensor_msgs::PointField::UINT16:
            output = readUnaligned<std::uint16_t>(point_address + field.offset);
            return true;
        case sensor_msgs::PointField::INT32:
            output = readUnaligned<std::int32_t>(point_address + field.offset);
            return true;
        case sensor_msgs::PointField::UINT32:
            output = readUnaligned<std::uint32_t>(point_address + field.offset);
            return true;
        case sensor_msgs::PointField::FLOAT32:
            output = readUnaligned<float>(point_address + field.offset);
            return true;
        case sensor_msgs::PointField::FLOAT64:
            output = readUnaligned<double>(point_address + field.offset);
            return true;
        default:
            return false;
    }
}

bool fieldFitsPoint(
    const sensor_msgs::PointField* field,
    std::uint32_t point_step)
{
    return field != nullptr && dataTypeSize(field->datatype) > 0U &&
        field->offset + dataTypeSize(field->datatype) <= point_step;
}

bool convertPointCloud2(
    const sensor_msgs::PointCloud2& message,
    PointCloud::Ptr& output)
{
    if (message.is_bigendian)
    {
        ROS_ERROR_THROTTLE(1.0, "Big-endian PointCloud2 is not supported");
        return false;
    }

    const sensor_msgs::PointField* x_field = findField(message, "x");
    const sensor_msgs::PointField* y_field = findField(message, "y");
    const sensor_msgs::PointField* z_field = findField(message, "z");
    const sensor_msgs::PointField* intensity_field = findField(message, "intensity");
    const sensor_msgs::PointField* ring_field = findField(message, "ring");
    const sensor_msgs::PointField* time_field = findField(message, "time");
    if (time_field == nullptr)
    {
        time_field = findField(message, "timestamp");
    }

    if (!fieldFitsPoint(x_field, message.point_step) ||
        !fieldFitsPoint(y_field, message.point_step) ||
        !fieldFitsPoint(z_field, message.point_step))
    {
        ROS_ERROR_THROTTLE(
            1.0, "Input PointCloud2 must contain valid x, y and z fields");
        return false;
    }

    if (!fieldFitsPoint(intensity_field, message.point_step))
    {
        intensity_field = nullptr;
        ROS_WARN_THROTTLE(5.0, "Input cloud has no usable intensity field");
    }
    if (!fieldFitsPoint(ring_field, message.point_step))
    {
        ring_field = nullptr;
        ROS_WARN_THROTTLE(5.0, "Input cloud has no usable ring field");
    }
    if (!fieldFitsPoint(time_field, message.point_step))
    {
        time_field = nullptr;
        ROS_WARN_THROTTLE(
            5.0, "Input cloud has neither a usable time nor timestamp field");
    }
    if (message.point_step == 0U || message.row_step == 0U)
    {
        return false;
    }

    output.reset(new PointCloud);
    pcl_conversions::toPCL(message.header, output->header);
    output->reserve(static_cast<std::size_t>(message.width) * message.height);

    for (std::uint32_t row = 0U; row < message.height; ++row)
    {
        for (std::uint32_t column = 0U; column < message.width; ++column)
        {
            const std::size_t offset = static_cast<std::size_t>(row) * message.row_step +
                static_cast<std::size_t>(column) * message.point_step;
            if (offset + message.point_step > message.data.size())
            {
                ROS_ERROR_THROTTLE(1.0, "Malformed PointCloud2 data buffer");
                return false;
            }

            const std::uint8_t* point_address = message.data.data() + offset;
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            readNumericField(point_address, *x_field, x);
            readNumericField(point_address, *y_field, y);
            readNumericField(point_address, *z_field, z);

            PointT point;
            point.x = static_cast<float>(x);
            point.y = static_cast<float>(y);
            point.z = static_cast<float>(z);
            point.data[3] = 1.0F;
            double value = 0.0;
            if (intensity_field && readNumericField(point_address, *intensity_field, value))
            {
                point.intensity = static_cast<float>(value);
            }
            if (ring_field && readNumericField(point_address, *ring_field, value))
            {
                value = std::max(0.0, std::min(65535.0, value));
                point.ring = static_cast<std::uint16_t>(value);
            }
            if (time_field && readNumericField(point_address, *time_field, value))
            {
                point.time = value;
            }
            output->push_back(point);
        }
    }

    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1U;
    output->is_dense = message.is_dense;
    return true;
}

}  // namespace

class LidarConeDetectorNode
{
public:
    LidarConeDetectorNode()
        : private_nh_("~")
    {
        std::string input_topic;
        std::string cluster_debug_topic;
        std::string candidate_debug_topic;
        std::string positions_debug_topic;
        std::string output_topic;
        int queue_size = 1;

        private_nh_.param<std::string>(
            "input_topic", input_topic, "/lidar/non_ground_points");
        private_nh_.param<std::string>(
            "cluster_debug_topic", cluster_debug_topic,
            "/lidar/debug/clustered_points");
        private_nh_.param<std::string>(
            "candidate_debug_topic", candidate_debug_topic,
            "/lidar/debug/cone_candidate_points");
        private_nh_.param<std::string>(
            "positions_debug_topic", positions_debug_topic,
            "/lidar/debug/cone_positions");
        private_nh_.param<std::string>(
            "output_topic", output_topic, "/perception/lidar/cones_raw");
        private_nh_.param("queue_size", queue_size, 1);
        private_nh_.param("publish_debug", publish_debug_, true);

        const GeometricFeatureExtractorConfig feature_config = loadFeatureConfig();
        clusterer_.reset(new AdaptiveEuclideanClusterer(
            loadRangeBands(),
            static_cast<float>(feature_config.lidar_origin_x),
            static_cast<float>(feature_config.lidar_origin_y)));
        feature_extractor_.reset(new GeometricFeatureExtractor(feature_config));
        classifier_.reset(new ConeClusterClassifier(loadClassifierConfig()));
        position_estimator_.reset(new ConePositionEstimator(
            loadPositionConfig(feature_config)));

        cloud_subscriber_ = nh_.subscribe(
            input_topic,
            std::max(1, queue_size),
            &LidarConeDetectorNode::cloudCallback,
            this);
        cone_publisher_ =
            nh_.advertise<driverless_msgs::ConeObservationArray>(output_topic, 1);
        cluster_debug_publisher_ =
            nh_.advertise<sensor_msgs::PointCloud2>(cluster_debug_topic, 1);
        candidate_debug_publisher_ =
            nh_.advertise<sensor_msgs::PointCloud2>(candidate_debug_topic, 1);
        positions_debug_publisher_ =
            nh_.advertise<geometry_msgs::PoseArray>(positions_debug_topic, 1);

        ROS_INFO_STREAM(
            "LiDAR cone detector started. input=" << input_topic
            << ", output=" << output_topic);
    }

private:
    struct Detection
    {
        std::size_t cluster_index = 0U;
        ClusterFeatures features;
        ConeClassification classification;
        ConePositionEstimate position;
    };

    RangeBandConfig loadBand(
        const std::string& name,
        float min_range,
        float max_range,
        float tolerance,
        int min_size,
        int max_size)
    {
        RangeBandConfig band;
        band.name = name;
        const std::string prefix = "bands/" + name + "/";
        private_nh_.param(prefix + "min_range", band.min_range, min_range);
        private_nh_.param(prefix + "max_range", band.max_range, max_range);
        private_nh_.param(
            prefix + "cluster_tolerance", band.cluster_tolerance, tolerance);
        private_nh_.param(prefix + "min_cluster_size", band.min_cluster_size, min_size);
        private_nh_.param(prefix + "max_cluster_size", band.max_cluster_size, max_size);
        return band;
    }

    std::vector<RangeBandConfig> loadRangeBands()
    {
        std::vector<RangeBandConfig> bands;
        bands.push_back(loadBand("near", 0.5F, 5.0F, 0.10F, 4, 500));
        bands.push_back(loadBand("middle", 5.0F, 10.0F, 0.15F, 3, 300));
        bands.push_back(loadBand("far", 10.0F, 15.0F, 0.22F, 2, 200));
        bands.push_back(loadBand("very_far", 15.0F, 20.0F, 0.30F, 2, 100));
        return bands;
    }

    GeometricFeatureExtractorConfig loadFeatureConfig()
    {
        GeometricFeatureExtractorConfig config;
        private_nh_.param(
            "features/lidar_origin_x", config.lidar_origin_x, 0.0);
        private_nh_.param(
            "features/lidar_origin_y", config.lidar_origin_y, 0.0);
        int pca_points = 5;
        int slice_points = 2;
        private_nh_.param("features/min_points_for_pca", pca_points, 5);
        private_nh_.param("features/min_points_per_slice", slice_points, 2);
        config.min_points_for_pca = static_cast<std::size_t>(std::max(3, pca_points));
        config.min_points_per_slice = static_cast<std::size_t>(std::max(2, slice_points));
        private_nh_.param(
            "features/bottom_slice_limit", config.bottom_slice_limit, 0.35);
        private_nh_.param(
            "features/middle_slice_limit", config.middle_slice_limit, 0.70);
        return config;
    }

    ConeClassifierConfig loadClassifierConfig()
    {
        ConeClassifierConfig config;
        const char* band_names[4] = {"near", "middle", "far", "very_far"};
        for (std::size_t index = 0U; index < 4U; ++index)
        {
            int min_points = static_cast<int>(config.min_points_by_band[index]);
            int reference_points = static_cast<int>(config.reference_points_by_band[index]);
            private_nh_.param(
                std::string("classifier/min_points/") + band_names[index],
                min_points,
                min_points);
            private_nh_.param(
                std::string("classifier/reference_points/") + band_names[index],
                reference_points,
                reference_points);
            config.min_points_by_band[index] =
                static_cast<std::size_t>(std::max(1, min_points));
            config.reference_points_by_band[index] =
                static_cast<std::size_t>(std::max(min_points + 1, reference_points));
        }

        private_nh_.param("classifier/height/hard_min", config.hard_min_height, 0.05);
        private_nh_.param("classifier/height/preferred_min", config.preferred_min_height, 0.15);
        private_nh_.param("classifier/height/preferred_max", config.preferred_max_height, 0.50);
        private_nh_.param("classifier/height/hard_max", config.hard_max_height, 0.75);
        private_nh_.param("classifier/width/hard_min", config.hard_min_width, 0.02);
        private_nh_.param("classifier/width/preferred_min", config.preferred_min_width, 0.06);
        private_nh_.param("classifier/width/preferred_max", config.preferred_max_width, 0.40);
        private_nh_.param("classifier/width/hard_max", config.hard_max_width, 0.60);
        private_nh_.param("classifier/depth/hard_min", config.hard_min_depth, 0.005);
        private_nh_.param("classifier/depth/preferred_min", config.preferred_min_depth, 0.02);
        private_nh_.param("classifier/depth/preferred_max", config.preferred_max_depth, 0.30);
        private_nh_.param("classifier/depth/hard_max", config.hard_max_depth, 0.50);
        private_nh_.param("classifier/use_ground_check", config.use_ground_check, false);
        private_nh_.param("classifier/expected_ground_z", config.expected_ground_z, 0.0);
        private_nh_.param("classifier/use_ring_score", config.use_ring_score, true);
        private_nh_.param(
            "classifier/minimum_structural_score",
            config.minimum_structural_score,
            0.35);
        private_nh_.param(
            "classifier/strong_confidence_threshold",
            config.strong_confidence_threshold,
            0.70);
        return config;
    }

    ConePositionEstimatorConfig loadPositionConfig(
        const GeometricFeatureExtractorConfig& feature_config)
    {
        ConePositionEstimatorConfig config;
        config.lidar_origin_x = feature_config.lidar_origin_x;
        config.lidar_origin_y = feature_config.lidar_origin_y;
        private_nh_.param(
            "position/nominal_cone_radius", config.nominal_cone_radius, 0.11);
        private_nh_.param(
            "position/minimum_cone_radius", config.minimum_cone_radius, 0.04);
        private_nh_.param(
            "position/maximum_cone_radius", config.maximum_cone_radius, 0.25);
        private_nh_.param(
            "position/measured_radius_weight", config.measured_radius_weight, 0.70);
        private_nh_.param(
            "position/near_surface_quantile", config.near_surface_quantile, 0.15);
        private_nh_.param(
            "position/use_fixed_ground_z", config.use_fixed_ground_z, false);
        private_nh_.param(
            "position/fixed_ground_z", config.fixed_ground_z, 0.0);
        private_nh_.param("position/base_z_offset", config.base_z_offset, 0.0);
        return config;
    }

    void publishClusterDebug(
        const std::vector<ClusterResult>& clusters,
        const std_msgs::Header& header)
    {
        static const std::uint8_t colors[][3] = {
            {255U, 64U, 64U}, {64U, 255U, 64U}, {64U, 64U, 255U},
            {255U, 255U, 64U}, {255U, 64U, 255U}, {64U, 255U, 255U},
            {255U, 160U, 64U}, {160U, 64U, 255U}};
        const std::size_t color_count = sizeof(colors) / sizeof(colors[0]);
        pcl::PointCloud<pcl::PointXYZRGB> debug_cloud;
        pcl_conversions::toPCL(header, debug_cloud.header);
        for (const auto& cluster : clusters)
        {
            const std::uint8_t* color = colors[cluster.id % color_count];
            for (const auto& input_point : cluster.cloud->points)
            {
                pcl::PointXYZRGB point;
                point.x = input_point.x;
                point.y = input_point.y;
                point.z = input_point.z;
                point.r = color[0];
                point.g = color[1];
                point.b = color[2];
                debug_cloud.push_back(point);
            }
        }
        debug_cloud.width = static_cast<std::uint32_t>(debug_cloud.size());
        debug_cloud.height = 1U;
        debug_cloud.is_dense = true;
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(debug_cloud, output);
        output.header = header;
        cluster_debug_publisher_.publish(output);
    }

    void publishCandidateDebug(
        const std::vector<ClusterResult>& clusters,
        const std::vector<Detection>& detections,
        const std_msgs::Header& header)
    {
        pcl::PointCloud<pcl::PointXYZRGB> debug_cloud;
        pcl_conversions::toPCL(header, debug_cloud.header);
        for (const Detection& detection : detections)
        {
            const bool strong =
                detection.classification.level == ConeCandidateLevel::STRONG;
            const ClusterResult& cluster = clusters[detection.cluster_index];
            for (const PointT& input_point : cluster.cloud->points)
            {
                pcl::PointXYZRGB point;
                point.x = input_point.x;
                point.y = input_point.y;
                point.z = input_point.z;
                point.r = strong ? 32U : 255U;
                point.g = strong ? 255U : 165U;
                point.b = 32U;
                debug_cloud.push_back(point);
            }
        }
        debug_cloud.width = static_cast<std::uint32_t>(debug_cloud.size());
        debug_cloud.height = 1U;
        debug_cloud.is_dense = true;
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(debug_cloud, output);
        output.header = header;
        candidate_debug_publisher_.publish(output);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message)
    {
        const ros::WallTime start_time = ros::WallTime::now();
        PointCloud::Ptr input_cloud;
        if (!convertPointCloud2(*message, input_cloud))
        {
            return;
        }

        const std::vector<ClusterResult> clusters = clusterer_->cluster(input_cloud);
        std::vector<Detection> detections;
        driverless_msgs::ConeObservationArray output;
        output.header = message->header;
        geometry_msgs::PoseArray pose_output;
        pose_output.header = message->header;
        std::size_t weak_count = 0U;
        std::size_t strong_count = 0U;

        for (std::size_t index = 0U; index < clusters.size(); ++index)
        {
            Detection detection;
            detection.cluster_index = index;
            detection.features = feature_extractor_->extract(clusters[index]);
            detection.classification = classifier_->classify(detection.features);
            if (!detection.classification.is_candidate)
            {
                continue;
            }
            detection.position = position_estimator_->estimate(
                clusters[index],
                detection.features,
                detection.classification);
            if (!detection.position.valid)
            {
                continue;
            }

            driverless_msgs::ConeObservation cone;
            cone.header = message->header;
            cone.id = clusters[index].id;
            cone.position.x = detection.position.x;
            cone.position.y = detection.position.y;
            cone.position.z = detection.position.z;
            for (std::size_t covariance_index = 0U;
                 covariance_index < detection.position.covariance.size();
                 ++covariance_index)
            {
                cone.position_covariance[covariance_index] =
                    detection.position.covariance[covariance_index];
            }
            cone.source = driverless_msgs::ConeObservation::SOURCE_LIDAR;
            cone.semantic_class =
                driverless_msgs::ConeObservation::SEMANTIC_UNKNOWN;
            cone.candidate_level =
                detection.classification.level == ConeCandidateLevel::STRONG ?
                driverless_msgs::ConeObservation::CANDIDATE_STRONG :
                driverless_msgs::ConeObservation::CANDIDATE_WEAK;
            cone.existence_probability = static_cast<float>(
                detection.classification.confidence);
            cone.lidar_confidence = cone.existence_probability;
            cone.camera_color_confidence = 0.0F;
            cone.height = static_cast<float>(detection.features.height);
            cone.width = static_cast<float>(detection.features.tangential_width);
            cone.depth = static_cast<float>(detection.features.radial_depth);
            cone.point_count = static_cast<std::uint32_t>(detection.features.point_count);
            cone.band_index = static_cast<std::uint8_t>(
                std::min<std::size_t>(255U, detection.features.band_index));
            cone.confirmed = false;
            output.cones.push_back(cone);

            geometry_msgs::Pose pose;
            pose.position = cone.position;
            pose.orientation.w = 1.0;
            pose_output.poses.push_back(pose);

            if (detection.classification.level == ConeCandidateLevel::STRONG)
            {
                ++strong_count;
            }
            else
            {
                ++weak_count;
            }
            detections.push_back(detection);
        }

        cone_publisher_.publish(output);
        if (publish_debug_)
        {
            publishClusterDebug(clusters, message->header);
            publishCandidateDebug(clusters, detections, message->header);
            positions_debug_publisher_.publish(pose_output);
        }

        const double elapsed_ms =
            (ros::WallTime::now() - start_time).toSec() * 1000.0;
        ROS_INFO_STREAM_THROTTLE(
            1.0,
            "cone_detection input_points=" << input_cloud->size()
            << ", clusters=" << clusters.size()
            << ", weak=" << weak_count
            << ", strong=" << strong_count
            << ", elapsed_ms=" << elapsed_ms);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber cloud_subscriber_;
    ros::Publisher cone_publisher_;
    ros::Publisher cluster_debug_publisher_;
    ros::Publisher candidate_debug_publisher_;
    ros::Publisher positions_debug_publisher_;
    std::unique_ptr<AdaptiveEuclideanClusterer> clusterer_;
    std::unique_ptr<GeometricFeatureExtractor> feature_extractor_;
    std::unique_ptr<ConeClusterClassifier> classifier_;
    std::unique_ptr<ConePositionEstimator> position_estimator_;
    bool publish_debug_ = true;
};

}  // namespace lidar_cone_detector

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_cone_detector_node");
    try
    {
        lidar_cone_detector::LidarConeDetectorNode node;
        ros::spin();
    }
    catch (const std::exception& exception)
    {
        ROS_FATAL("Failed to start lidar_cone_detector_node: %s", exception.what());
        return 1;
    }
    return 0;
}
