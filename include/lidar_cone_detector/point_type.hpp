#pragma once

#include <cstdint>

#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

namespace lidar_cone_detector
{

struct EIGEN_ALIGN16 PointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity = 0.0F;
    std::uint16_t ring = 0U;
    double time = 0.0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  //lidar_cone_detector end  

POINT_CLOUD_REGISTER_POINT_STRUCT(
    lidar_cone_detector::PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, time, time))
