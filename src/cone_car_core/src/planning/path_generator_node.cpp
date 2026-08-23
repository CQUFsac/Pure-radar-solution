#include <ros/ros.h>
#include <cone_car_core/Cone.h>
#include <cone_car_core/ConeArray.h>
#include <cone_car_core/PathPoint.h>
#include <cone_car_core/PlanningPath.h>
#include <cone_car_core/VehicleState.h>
#include <algorithm>
#include <cmath>

class PathGeneratorNode
{
public:
    PathGeneratorNode()
    {
        cone_sub_ = nh_.subscribe("/perception/cones", 10, &PathGeneratorNode::coneCallback, this);
        state_sub_ = nh_.subscribe("/localization/vehicle_state", 10, &PathGeneratorNode::stateCallback, this);
        pub_ = nh_.advertise<cone_car_core::PlanningPath>("/planning/path", 10);

        nh_.param<int>("planning/smoothing_window", smoothing_window_, 3);
        nh_.param<float>("perception/max_detection_distance", max_dist_, 10.0f);
        nh_.param<int>("planning/min_centerline_points", min_points_, 3);
        nh_.param<float>("planning/max_boundary_gap", max_gap_, 3.0f);

        ROS_INFO("[PathGeneratorNode] Initialized");
    }

private:
    void stateCallback(const cone_car_core::VehicleState::ConstPtr& msg)
    {
        vehicle_state_ = *msg;
    }

    void coneCallback(const cone_car_core::ConeArray::ConstPtr& msg)
    {
        cone_car_core::PlanningPath path;
        path.header = msg->header;
        path.path_valid = false;
        path.quality = 0.0f;

        // Store raw cones
        path.raw_cones = msg->cones;

        // Step 1: Filter cones (x > 0, within max distance)
        std::vector<cone_car_core::Cone> filtered;
        for (const auto& cone : msg->cones)
        {
            if (cone.x > 0.0f && cone.x < max_dist_)
            {
                filtered.push_back(cone);
            }
        }
        path.filtered_cones = filtered;

        // Step 2: Separate into left / right boundaries
        // 有颜色用颜色；纯雷达(unknown)用 y 符号分边（base_link 帧 y>0=左）
        std::vector<cone_car_core::Cone> left_cones, right_cones;
        bool has_color = false;
        for (const auto& cone : filtered)
        {
            if (cone.color == "blue" || cone.color == "yellow") { has_color = true; break; }
        }

        for (const auto& cone : filtered)
        {
            if (has_color)
            {
                if (cone.color == "blue")        left_cones.push_back(cone);
                else if (cone.color == "yellow") right_cones.push_back(cone);
                else { (cone.y > 0.0f ? left_cones : right_cones).push_back(cone); }
            }
            else
            {
                // 纯雷达无颜色：按 y 符号分边
                (cone.y > 0.0f ? left_cones : right_cones).push_back(cone);
            }
        }

        // Step 3: Sort by x
        std::sort(left_cones.begin(), left_cones.end(),
                  [](const cone_car_core::Cone& a, const cone_car_core::Cone& b) { return a.x < b.x; });
        std::sort(right_cones.begin(), right_cones.end(),
                  [](const cone_car_core::Cone& a, const cone_car_core::Cone& b) { return a.x < b.x; });

        // Step 4: Build boundary PathPoints
        for (const auto& c : left_cones)
        {
            cone_car_core::PathPoint p;
            p.x = c.x;
            p.y = c.y;
            p.curvature = 0.0f;
            p.target_speed = 0.0f;
            path.left_boundary.push_back(p);
        }
        for (const auto& c : right_cones)
        {
            cone_car_core::PathPoint p;
            p.x = c.x;
            p.y = c.y;
            p.curvature = 0.0f;
            p.target_speed = 0.0f;
            path.right_boundary.push_back(p);
        }

        // Step 5: Generate centerline — 最近邻取中点（适配左右数量不等/八字弯）
        // 对每个左锥桶，找最近的右锥桶取中点；按 x 排序保证轨迹有序
        if (!left_cones.empty() && !right_cones.empty())
        {
            for (const auto& lc : left_cones)
            {
                float best = max_gap_;
                int best_j = -1;
                for (size_t j = 0; j < right_cones.size(); ++j)
                {
                    float d = std::hypot(lc.x - right_cones[j].x, lc.y - right_cones[j].y);
                    if (d < best) { best = d; best_j = (int)j; }
                }
                if (best_j >= 0)
                {
                    cone_car_core::PathPoint p;
                    p.x = (lc.x + right_cones[best_j].x) / 2.0f;
                    p.y = (lc.y + right_cones[best_j].y) / 2.0f;
                    p.curvature = 0.0f;
                    p.target_speed = 0.0f;
                    path.centerline.push_back(p);
                }
            }
            // 按 x 排序，保证中心线沿前进方向有序
            std::sort(path.centerline.begin(), path.centerline.end(),
                      [](const cone_car_core::PathPoint& a, const cone_car_core::PathPoint& b) {
                          return a.x < b.x;
                      });
        }
        else if (!left_cones.empty() || !right_cones.empty())
        {
            // 只有单侧边界：沿该侧偏移生成中心线（兜底，直线段常见）
            const auto& side = left_cones.empty() ? right_cones : left_cones;
            float sign = left_cones.empty() ? 1.0f : -1.0f;  // 只有右→中心线在左
            const float half_track = 1.5f;  // 半赛道宽度估计 (m)
            for (const auto& c : side)
            {
                cone_car_core::PathPoint p;
                p.x = c.x;
                p.y = c.y + sign * half_track;
                p.curvature = 0.0f;
                p.target_speed = 0.0f;
                path.centerline.push_back(p);
            }
        }

        // Step 6: Compute curvature for centerline points
        computeCurvature(path.centerline);

        // Step 7: Simple smoothing (moving average)
        smoothPath(path.centerline, smoothing_window_);

        // Step 7.5: 平滑后重算曲率（坐标已变，曲率需同步更新）
        computeCurvature(path.centerline);

        // Step 8: Validate path
        if ((int)path.centerline.size() >= min_points_)
        {
            path.path_valid = true;
            path.quality = std::min(1.0f, (float)path.centerline.size() / 5.0f);
        }

        pub_.publish(path);
    }

    void computeCurvature(std::vector<cone_car_core::PathPoint>& pts)
    {
        if (pts.size() < 3) return;
        for (size_t i = 1; i < pts.size() - 1; ++i)
        {
            float dx1 = pts[i].x - pts[i - 1].x;
            float dy1 = pts[i].y - pts[i - 1].y;
            float dx2 = pts[i + 1].x - pts[i].x;
            float dy2 = pts[i + 1].y - pts[i].y;

            float cross = dx1 * dy2 - dy1 * dx2;
            float len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
            float len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
            float len_avg = (len1 + len2) / 2.0f;

            if (len_avg > 1e-4f)
            {
                pts[i].curvature = 2.0f * cross / (len1 * len2 * len_avg);
            }
        }
    }

    void smoothPath(std::vector<cone_car_core::PathPoint>& pts, int window)
    {
        if ((int)pts.size() <= window) return;
        std::vector<cone_car_core::PathPoint> smoothed = pts;
        int half = window / 2;
        for (int i = half; i < (int)pts.size() - half; ++i)
        {
            float sum_x = 0.0f, sum_y = 0.0f;
            for (int j = -half; j <= half; ++j)
            {
                sum_x += pts[i + j].x;
                sum_y += pts[i + j].y;
            }
            smoothed[i].x = sum_x / window;
            smoothed[i].y = sum_y / window;
        }
        pts = smoothed;
    }

    ros::NodeHandle nh_;
    ros::Subscriber cone_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher pub_;
    cone_car_core::VehicleState vehicle_state_;

    int smoothing_window_;
    float max_dist_;
    int min_points_;
    float max_gap_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_generator_node");
    PathGeneratorNode node;
    ros::spin();
    return 0;
}