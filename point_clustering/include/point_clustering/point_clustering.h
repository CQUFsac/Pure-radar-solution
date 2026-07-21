#ifndef POINT_CLUSTERING_H
#define POINT_CLUSTERING_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <vector>

// RoboSense point type (same definition as in roi_filter)
struct RsPointXYZIRT
{
  PCL_ADD_POINT4D;
  uint8_t intensity;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint8_t, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

class PointClusteringNode
{
public:
    PointClusteringNode();
    ~PointClusteringNode() = default;

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud);
    void publishClusters(const std::vector<RsPointXYZIRT>& cone_point,
                         const std_msgs::Header& header);
    bool seedpoint_convergence_check(const std::vector<RsPointXYZIRT>& old_seed,
                                     const std::vector<RsPointXYZIRT>& new_seed,
                                     double threshold);

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher  pub_;

    std::string input_topic_;
    std::string output_topic_;

    // Seed point clustering parameters
    double seedpoint_carDistance_;
    double point_pointDistance_;
    float  roi_[4];
    int    seednub_gain_;

    // Seed point data
    int    seedpoint_number_;
    double seedpoint_pointDistance_;
    std::vector<RsPointXYZIRT> seed_point[2];
    std::vector<RsPointXYZIRT> seed_point_old[2];
    std::vector<RsPointXYZIRT> cone_point[2];
};

#endif // POINT_CLUSTERING_H