/**
 * cone_topic_converter.cpp
 *
 * 将 driverless_msgs::ConeObservationArray 转换为 cone_car_core::ConeArray
 * 让 cone_slam 能接收来自 fssim_bridge 的锥桶数据
 *
 * 订阅: /perception/lidar/cones_raw (driverless_msgs/ConeObservationArray)
 * 发布: /perception/cones (cone_car_core/ConeArray)
 */

#include <ros/ros.h>
#include <driverless_msgs/ConeObservationArray.h>
#include <driverless_msgs/ConeObservation.h>
#include <cone_car_core/ConeArray.h>
#include <cone_car_core/Cone.h>

class ConeTopicConverter
{
public:
    ConeTopicConverter()
    {
        cone_sub_ = nh_.subscribe("/perception/lidar/cones_raw", 1,
            &ConeTopicConverter::coneCallback, this);
        cone_pub_ = nh_.advertise<cone_car_core::ConeArray>("/perception/cones", 1);
        ROS_INFO("ConeTopicConverter started: /perception/lidar/cones_raw → /perception/cones");
    }

private:
    void coneCallback(const driverless_msgs::ConeObservationArray::ConstPtr& msg)
    {
        cone_car_core::ConeArray output;
        output.header = msg->header;

        for (const auto& obs : msg->cones)
        {
            cone_car_core::Cone cone;
            cone.id = obs.id;
            cone.x = obs.position.x;
            cone.y = obs.position.y;
            cone.z = obs.position.z;

            // 颜色映射: 1=yellow/left, 2=blue/right, 3=orange
            switch (obs.semantic_class)
            {
                case 1:  cone.color = "yellow"; break;
                case 2:  cone.color = "blue";   break;
                case 3:  cone.color = "orange"; break;
                default: cone.color = "unknown"; break;
            }

            cone.confidence = obs.existence_probability;
            cone.source = "lidar";
            cone.age = 1;
            output.cones.push_back(cone);
        }

        cone_pub_.publish(output);
    }

    ros::NodeHandle nh_;
    ros::Subscriber cone_sub_;
    ros::Publisher cone_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cone_topic_converter");
    ConeTopicConverter converter;
    ros::spin();
    return 0;
}