/**
 * fssim_odom_bridge.cpp
 *
 * 桥接 fssim 仿真器的 topic 到系统期望的格式:
 *   - /fssim/state (fssim_common/State) → /odometry/filtered (nav_msgs/Odometry)
 *   - TF: odom → base_link
 *   - TF: base_link → fssim/vehicle/base_link (identity, for frame compatibility)
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fssim_common/State.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

class FssimOdomBridge
{
public:
    FssimOdomBridge() : nh_("~")
    {
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odometry/filtered", 50);
        state_sub_ = nh_.subscribe("/fssim/state", 50, &FssimOdomBridge::stateCallback, this);
        ROS_INFO("FssimOdomBridge started: /fssim/state → /odometry/filtered + TF");
    }

private:
    void stateCallback(const fssim_common::State::ConstPtr& msg)
    {
        ros::Time now = ros::Time::now();

        // Publish Odometry
        nav_msgs::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = msg->x;
        odom.pose.pose.position.y = msg->y;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, msg->yaw);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = msg->vx;
        odom.twist.twist.linear.y = msg->vy;
        odom.twist.twist.angular.z = msg->r;

        odom_pub_.publish(odom);

        // Publish TF: odom → base_link
        geometry_msgs::TransformStamped tf_odom;
        tf_odom.header.stamp = now;
        tf_odom.header.frame_id = "odom";
        tf_odom.child_frame_id = "base_link";
        tf_odom.transform.translation.x = msg->x;
        tf_odom.transform.translation.y = msg->y;
        tf_odom.transform.translation.z = 0.0;
        tf_odom.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_.sendTransform(tf_odom);
    }

    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber state_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fssim_odom_bridge");
    FssimOdomBridge bridge;
    ros::spin();
    return 0;
}