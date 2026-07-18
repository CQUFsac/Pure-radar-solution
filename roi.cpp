#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>


float ROI[6]; 

struct Point {
    float x, y, z, intensity;
};

bool insideBox(const Point& p,
               float xmin, float xmax,
               float ymin, float ymax,
               float zmin, float zmax)
{
    return p.x >= xmin && p.x <= xmax &&
           p.y >= ymin && p.y <= ymax &&
           p.z >= zmin && p.z <= zmax;
}

int main() {
    std::vector<Point> cloud;

    // 预分配（避免反复 realloc）
    //cloud.reserve(100000);

    // 塞数据
    cloud.push_back({1.0f, 2.0f, 3.0f, 0.5f});
    cloud.push_back({4.0f, 5.0f, 6.0f, 0.8f});

    // ROI 过滤：只保留盒子内的点（去掉盒子外的点）
    std::vector<Point> filtered;
    filtered.reserve(cloud.size());
    for (const auto& p : cloud) {
        if (insideBox(p, ROI[0], ROI[1], ROI[2], ROI[3], ROI[4], ROI[5])) {
            filtered.push_back(p);
        }
    }
    cloud = std::move(filtered);
}