“”“
    对脏点云数据进行 ROI（Region of Interest）过滤和地面点提取的 ROS 节点。
    处理结果：
      1. ROI 点云：只保留指定 ROI 范围内的点云数据，发布到 roi_topic
      2. 地面点云：提取 z 低于地面阈值的点，发布到 ground_topic
    ROI 形状为长方形，及保留车辆前方的点云数据。
    
    使用 RsPointXYZIRT 点类型，保留 x, y, z, intensity, ring, timestamp 所有字段。
”“”


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

// RoboSense 点类型定义（与 rslidar_sdk 中的 RsPointXYZIRT 一致）
// 包含 x, y, z, intensity, ring, timestamp 六个字段
struct RsPointXYZIRT
{
  PCL_ADD_POINT4D;          // x, y, z (4 floats, 16 bytes)
  uint8_t intensity;        // 反射强度 (0-255)
  uint16_t ring = 0;        // 激光线束编号
  double timestamp = 0;     // 时间戳（秒）
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

class RoiFilterNode
{
public:
    RoiFilterNode() : nh_("~")
    {
        // 1. 从参数服务器读取 ROI 参数（带默认值）
        nh_.param<float>("roi_xmin", roi_[0], -50.0f);
        nh_.param<float>("roi_xmax", roi_[1],  50.0f);
        nh_.param<float>("roi_ymin", roi_[2], -50.0f);
        nh_.param<float>("roi_ymax", roi_[3],  50.0f);
        nh_.param<float>("roi_zmin", roi_[4],  -5.0f);
        nh_.param<float>("roi_zmax", roi_[5],   5.0f);

        // 2. 读取地面点高度阈值
        nh_.param<float>("ground_z_threshold", ground_z_threshold_, -0.5f);

        // 3. 读取 topic 名称参数
        std::string input_topic, roi_topic, ground_topic;
        nh_.param<std::string>("input_topic",   input_topic,   "/rslidar_points");
        nh_.param<std::string>("roi_topic",     roi_topic,     "/radar/points_roi");
        nh_.param<std::string>("ground_topic",  ground_topic,  "/radar/points_ground");

        // 4. 创建订阅者和发布者
        sub_ = nh_.subscribe(input_topic, 10, &RoiFilterNode::cloudCallback, this);
        pub_roi_ = nh_.advertise<sensor_msgs::PointCloud2>(roi_topic, 10);
        pub_ground_ = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic, 10);

        ROS_INFO("ROI Filter Node started.");
        ROS_INFO("ROI: x[%.1f, %.1f] y[%.1f, %.1f] z[%.1f, %.1f]",
                 roi_[0], roi_[1], roi_[2], roi_[3], roi_[4], roi_[5]);
        ROS_INFO("Ground threshold: z < %.2f", ground_z_threshold_);
        ROS_INFO("Subscribing to: %s",  input_topic.c_str());
        ROS_INFO("Publishing ROI: %s",  roi_topic.c_str());
        ROS_INFO("Publishing Ground: %s", ground_topic.c_str());
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud)
    {
        // 1. sensor_msgs → pcl::PointCloud<RsPointXYZIRT>
        //    保留所有字段：x, y, z, intensity, ring, timestamp
        pcl::PointCloud<RsPointXYZIRT>::Ptr cloud(new pcl::PointCloud<RsPointXYZIRT>);
        pcl::fromROSMsg(*input_cloud, *cloud);

        // 2. ROI 过滤 + 地面点提取（一次遍历完成）
        pcl::PointCloud<RsPointXYZIRT>::Ptr roi_cloud(new pcl::PointCloud<RsPointXYZIRT>);
        pcl::PointCloud<RsPointXYZIRT>::Ptr ground_cloud(new pcl::PointCloud<RsPointXYZIRT>);
        roi_cloud->reserve(cloud->size());
        ground_cloud->reserve(cloud->size());

        for (const auto& point : *cloud)
        {
            // 判断是否在 ROI 范围内
            if (point.x >= roi_[0] && point.x <= roi_[1] &&
                point.y >= roi_[2] && point.y <= roi_[3] &&
                point.z >= roi_[4] && point.z <= roi_[5])
            {
                roi_cloud->push_back(point);
            }

            // 判断是否为地面点（z 低于地面阈值）
            if (point.z < ground_z_threshold_)
            {
                ground_cloud->push_back(point);
            }
        }

        // 3. 发布 ROI 点云（保留 x, y, z, intensity, ring, timestamp）
        sensor_msgs::PointCloud2 output_roi;
        pcl::toROSMsg(*roi_cloud, output_roi);
        output_roi.header = input_cloud->header;
        pub_roi_.publish(output_roi);

        // 4. 发布地面点云（保留 x, y, z, intensity, ring, timestamp）
        sensor_msgs::PointCloud2 output_ground;
        pcl::toROSMsg(*ground_cloud, output_ground);
        output_ground.header = input_cloud->header;
        pub_ground_.publish(output_ground);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher  pub_roi_;
    ros::Publisher  pub_ground_;
    float roi_[6];
    float ground_z_threshold_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "roi_filter_node");
    RoiFilterNode node;
    ros::spin();
    return 0;
}