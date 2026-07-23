#include <ros/ros.h>
#include "Cone_Map_Manager/cone_map_manager.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cone_map_manager_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ROS_INFO("Starting Cone Map Manager Node...");

  ConeMapManager manager(nh, private_nh);

  ros::spin();

  return 0;
}