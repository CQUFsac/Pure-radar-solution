/* """
    点云聚类节点，使用骨架点迭代聚类算法对点云进行分割。
    输入：/radar/points_roi（ROI 滤波后的点云）
    输出：/radar/clusters（聚类后的点云，每个聚类用不同颜色标识）
""" */

#include <point_clustering/point_clustering.h>
#include <pcl/common/centroid.h>

PointClusteringNode::PointClusteringNode() : nh_("~")
{
    // 1. 读取参数
    nh_.param<std::string>("input_topic",  input_topic_,  "/radar/points_roi");
    nh_.param<std::string>("output_topic", output_topic_, "/radar/clusters");

    nh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.5);
    nh_.param<int>("min_cluster_size",     min_cluster_size_,  10);
    nh_.param<int>("max_cluster_size",     max_cluster_size_,  10000);
    nh_.param<double>("seedpoint_carDistance", seedpoint_carDistance_, 1.0);
    nh_.param<double>("point_pointDistance", point_pointDistance_, 1.5);
    nh_.param<float>("roi_ymax", roi_[3],  50.0f);
    nh_.param<int>("seednub_gain", seednub_gain_,  10);

    // 计算种子点数量和间距
    seedpoint_number_ = static_cast<int>(roi_[3] / point_pointDistance_ * seednub_gain_);
    seedpoint_pointDistance_ = point_pointDistance_ / seednub_gain_;

    // 预分配种子点容器
    seed_point[0].resize(seedpoint_number_);
    seed_point[1].resize(seedpoint_number_);
    seed_point_old[0].resize(seedpoint_number_);
    seed_point_old[1].resize(seedpoint_number_);
    cone_point[0].resize(seedpoint_number_);
    cone_point[1].resize(seedpoint_number_);

    // 2. 创建订阅者和发布者
    sub_ = nh_.subscribe(input_topic_, 10, &PointClusteringNode::cloudCallback, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);

    ROS_INFO("Point Clustering Node started.");
    ROS_INFO("Subscribing to: %s",  input_topic_.c_str());
    ROS_INFO("Publishing to: %s",   output_topic_.c_str());
    ROS_INFO("Cluster tolerance: %.2f, min size: %d, max size: %d",
             cluster_tolerance_, min_cluster_size_, max_cluster_size_);
}

void PointClusteringNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud)
{
    // 1. ROS → PCL 点云数据初始化
    pcl::PointCloud<RsPointXYZIRT>::Ptr cloud(new pcl::PointCloud<RsPointXYZIRT>);
    pcl::fromROSMsg(*input_cloud, *cloud);

    // 左锥桶点云
    pcl::PointCloud<RsPointXYZIRT>::Ptr left_cone_cloud(new pcl::PointCloud<RsPointXYZIRT>);
    left_cone_cloud->reserve(cloud->size());

    // 右锥桶点云
    pcl::PointCloud<RsPointXYZIRT>::Ptr right_cone_cloud(new pcl::PointCloud<RsPointXYZIRT>);
    right_cone_cloud->reserve(cloud->size());

    // 2. 区分左右锥桶点云
    for (const auto& point : *cloud)
    {
        if (point.x < 0)
        {
            left_cone_cloud->push_back(point);
        }
        else
        {
            right_cone_cloud->push_back(point);
        }
    }

    // 3. 种子点初始化
    for (int i = 0; i < seedpoint_number_; ++i)
    {
        // 左侧第 i 个种子点（左侧 y 为负方向）
        seed_point[0][i].x = -seedpoint_carDistance_;
        seed_point[0][i].y = -seedpoint_pointDistance_ * (i + 1);
        seed_point[0][i].z = 0;

        // 右侧第 i 个种子点
        seed_point[1][i].x = seedpoint_carDistance_;
        seed_point[1][i].y = seedpoint_pointDistance_ * (i + 1);
        seed_point[1][i].z = 0;
    }

    // 4. 迭代：Voronoi 分区 → 重心更新，直到收敛
    do
    {
        // 为每个种子点创建一个子点云
        std::vector<pcl::PointCloud<RsPointXYZIRT>::Ptr> seed_cloud_right(seedpoint_number_);
        std::vector<pcl::PointCloud<RsPointXYZIRT>::Ptr> seed_cloud_left(seedpoint_number_);
        for (int i = 0; i < seedpoint_number_; ++i)
        {
            seed_cloud_right[i].reset(new pcl::PointCloud<RsPointXYZIRT>);
            seed_cloud_left[i].reset(new pcl::PointCloud<RsPointXYZIRT>);
        }

        // 右侧点云 Voronoi 分区
        for (size_t i = 0; i < right_cone_cloud->size(); ++i)
        {
            const auto& point = right_cone_cloud->points[i];

            // 计算到所有右侧种子点的距离
            double min_dist = std::numeric_limits<double>::max();
            int nearest_seed_idx = 0;
            for (int j = 0; j < seedpoint_number_; ++j)
            {
                double dist = pcl::euclideanDistance(point, seed_point[1][j]);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    nearest_seed_idx = j;
                }
            }
            seed_cloud_right[nearest_seed_idx]->push_back(point);
        }

        // 左侧点云 Voronoi 分区
        for (size_t i = 0; i < left_cone_cloud->size(); ++i)
        {
            const auto& point = left_cone_cloud->points[i];

            double min_dist = std::numeric_limits<double>::max();
            int nearest_seed_idx = 0;
            for (int j = 0; j < seedpoint_number_; ++j)
            {
                double dist = pcl::euclideanDistance(point, seed_point[0][j]);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    nearest_seed_idx = j;
                }
            }
            seed_cloud_left[nearest_seed_idx]->push_back(point);
        }

        // 5. 保存旧种子点
        for (int i = 0; i < seedpoint_number_; ++i)
        {
            seed_point_old[0][i] = seed_point[0][i];
            seed_point_old[1][i] = seed_point[1][i];
        }

        // 6. 计算重心，更新种子点
        for (int i = 0; i < seedpoint_number_; ++i)
        {
            if (!seed_cloud_right[i]->empty())
            {
                Eigen::Vector4f centroid;
                pcl::computeCentroid(*seed_cloud_right[i], centroid);
                seed_point[1][i].x = centroid[0];
                seed_point[1][i].y = centroid[1];
                seed_point[1][i].z = centroid[2];
            }

            if (!seed_cloud_left[i]->empty())
            {
                Eigen::Vector4f centroid;
                pcl::computeCentroid(*seed_cloud_left[i], centroid);
                seed_point[0][i].x = centroid[0];
                seed_point[0][i].y = centroid[1];
                seed_point[0][i].z = centroid[2];
            }
        }
    }while (!seedpoint_convergence_check(seed_point_old[0], seed_point[0], 0.1) &&
           !seedpoint_convergence_check(seed_point_old[1], seed_point[1], 0.1));
    

    // 7. 构建cone_point点云，包含所有收敛后的种子点
    pcl::PointCloud<RsPointXYZIRT>::Ptr cone_point(new pcl::PointCloud<RsPointXYZIRT>);
    cone_point->reserve(seedpoint_number_ * 2);
    for (int i = 0; i < seedpoint_number_; ++i)
    {
        cone_point->push_back(seed_point[0][i]);
        cone_point->push_back(seed_point[1][i]);
    }

    // 8. 发布cone_point结果
    if (!cone_point->empty())
    {
        publishClusters(cone_point, input_cloud->header);
    }
}

// 种子点变化收敛判断逻辑
bool PointClusteringNode::seedpoint_convergence_check(
    const std::vector<RsPointXYZIRT>& old_seed,
    const std::vector<RsPointXYZIRT>& new_seed,
    double threshold)
{
    for (size_t i = 0; i < old_seed.size(); ++i)
    {
        double distance = pcl::euclideanDistance(old_seed[i], new_seed[i]);
        if (distance > threshold)
        {
            return false;  // 任意一个种子点变化超过阈值 → 未收敛
        }
    }
    return true;  // 所有种子点都在阈值内 → 已收敛
}

void PointClusteringNode::publishClusters(
    const std::vector<RsPointXYZIRT>& cone_point,
    const std_msgs::Header& header)
{
    // 发布
    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(*cone_point, output_cloud);
    output_cloud.header = header;
    pub_.publish(output_cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_clustering_node");
    PointClusteringNode node;
    ros::spin();
    return 0;
}