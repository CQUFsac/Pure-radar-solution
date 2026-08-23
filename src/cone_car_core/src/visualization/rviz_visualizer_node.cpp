#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cone_car_core/ConeArray.h>
#include <cone_car_core/PlanningPath.h>
#include <cone_car_core/ModeStatus.h>
#include <cone_car_core/ControlCommand.h>
#include <cone_car_core/VehicleState.h>

class RvizVisualizerNode
{
public:
    RvizVisualizerNode()
    {
        cones_sub_ = nh_.subscribe("/perception/cones", 10, &RvizVisualizerNode::conesCallback, this);
        path_sub_ = nh_.subscribe("/planning/path", 10, &RvizVisualizerNode::pathCallback, this);
        mode_sub_ = nh_.subscribe("/mode/status", 10, &RvizVisualizerNode::modeCallback, this);
        cmd_sub_ = nh_.subscribe("/control/command", 10, &RvizVisualizerNode::cmdCallback, this);
        state_sub_ = nh_.subscribe("/localization/vehicle_state", 10, &RvizVisualizerNode::stateCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization/markers", 10);

        ROS_INFO("[RvizVisualizerNode] Initialized");
    }

private:
    // ==================== Callbacks ====================
    void conesCallback(const cone_car_core::ConeArray::ConstPtr& msg)
    {
        cones_ = *msg;
        cones_received_ = true;
    }

    void pathCallback(const cone_car_core::PlanningPath::ConstPtr& msg)
    {
        path_ = *msg;
        path_received_ = true;
    }

    void modeCallback(const cone_car_core::ModeStatus::ConstPtr& msg)
    {
        mode_ = *msg;
        mode_received_ = true;
    }

    void cmdCallback(const cone_car_core::ControlCommand::ConstPtr& msg)
    {
        cmd_ = *msg;
        cmd_received_ = true;
    }

    void stateCallback(const cone_car_core::VehicleState::ConstPtr& msg)
    {
        state_ = *msg;
        state_received_ = true;

        // Publish markers on state update (fixed rate ~50Hz)
        publishMarkers();
    }

    // ==================== Marker Publishing ====================
    void publishMarkers()
    {
        visualization_msgs::MarkerArray markers;

        // Clear all previous markers
        visualization_msgs::Marker clear;
        clear.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(clear);

        int id = 0;

        if (cones_received_)
        {
            // Blue cones (left boundary) — BLUE color
            for (const auto& cone : cones_.cones)
            {
                if (cone.color == "blue")
                {
                    markers.markers.push_back(makeSphereMarker(
                        "blue_cones", id++, cone.x, cone.y, cone.z,
                        0.0f, 0.0f, 1.0f, 1.0f, 0.25f));
                }
            }

            // Yellow cones (right boundary) — YELLOW color
            for (const auto& cone : cones_.cones)
            {
                if (cone.color == "yellow")
                {
                    markers.markers.push_back(makeSphereMarker(
                        "yellow_cones", id++, cone.x, cone.y, cone.z,
                        1.0f, 1.0f, 0.0f, 1.0f, 0.25f));
                }
            }
        }

        if (path_received_ && path_.path_valid)
        {
            // Left boundary — BLUE line
            if (path_.left_boundary.size() >= 2)
            {
                markers.markers.push_back(makeLineStripMarker(
                    "left_boundary", id++, path_.left_boundary,
                    0.0f, 0.0f, 1.0f, 0.8f, 0.08f));
            }

            // Right boundary — YELLOW line
            if (path_.right_boundary.size() >= 2)
            {
                markers.markers.push_back(makeLineStripMarker(
                    "right_boundary", id++, path_.right_boundary,
                    1.0f, 1.0f, 0.0f, 0.8f, 0.08f));
            }

            // Centerline — GREEN line
            if (path_.centerline.size() >= 2)
            {
                markers.markers.push_back(makeLineStripMarker(
                    "centerline", id++, path_.centerline,
                    0.0f, 1.0f, 0.0f, 1.0f, 0.1f));
            }
        }

        if (cmd_received_)
        {
            // Target point — RED sphere
            markers.markers.push_back(makeSphereMarker(
                "target_point", id++, cmd_.target_point_x, cmd_.target_point_y, 0.0f,
                1.0f, 0.0f, 0.0f, 1.0f, 0.3f));
        }

        if (state_received_)
        {
            // Vehicle position — WHITE sphere
            markers.markers.push_back(makeSphereMarker(
                "vehicle_pos", id++, state_.x, state_.y, 0.0f,
                1.0f, 1.0f, 1.0f, 1.0f, 0.4f));

            // Vehicle heading arrow
            markers.markers.push_back(makeArrowMarker(
                "vehicle_heading", id++, state_.x, state_.y, state_.yaw,
                0.0f, 1.0f, 0.0f, 1.0f, 1.0f));
        }

        if (mode_received_)
        {
            // Mode text
            markers.markers.push_back(makeTextMarker(
                "mode_text", id++, mode_.current_mode, 0.0f, 0.0f, 2.0f,
                1.0f, 1.0f, 1.0f, 0.6f));
        }

        marker_pub_.publish(markers);
    }

    // ==================== Marker Factories ====================
    visualization_msgs::Marker makeSphereMarker(
        const std::string& ns, int id,
        float x, float y, float z,
        float r, float g, float b, float a,
        float scale)
    {
        visualization_msgs::Marker m;
        m.header.frame_id = "base_link";
        m.header.stamp = ros::Time::now();
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = z;
        m.pose.orientation.w = 1.0;
        m.scale.x = scale;
        m.scale.y = scale;
        m.scale.z = scale;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = a;
        return m;
    }

    visualization_msgs::Marker makeLineStripMarker(
        const std::string& ns, int id,
        const std::vector<cone_car_core::PathPoint>& points,
        float r, float g, float b, float a,
        float scale)
    {
        visualization_msgs::Marker m;
        m.header.frame_id = "base_link";
        m.header.stamp = ros::Time::now();
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.scale.x = scale;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = a;
        m.pose.orientation.w = 1.0;

        for (const auto& p : points)
        {
            geometry_msgs::Point gp;
            gp.x = p.x;
            gp.y = p.y;
            gp.z = 0.0;
            m.points.push_back(gp);
        }
        return m;
    }

    visualization_msgs::Marker makeArrowMarker(
        const std::string& ns, int id,
        float x, float y, float yaw,
        float r, float g, float b, float a,
        float length)
    {
        visualization_msgs::Marker m;
        m.header.frame_id = "base_link";
        m.header.stamp = ros::Time::now();
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::Marker::ARROW;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = 0.0;
        m.pose.orientation.z = std::sin(yaw / 2.0f);
        m.pose.orientation.w = std::cos(yaw / 2.0f);
        m.scale.x = length;
        m.scale.y = 0.1;
        m.scale.z = 0.1;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = a;
        return m;
    }

    visualization_msgs::Marker makeTextMarker(
        const std::string& ns, int id,
        const std::string& text,
        float x, float y, float z,
        float r, float g, float b, float a,
        float scale = 0.5f)
    {
        visualization_msgs::Marker m;
        m.header.frame_id = "base_link";
        m.header.stamp = ros::Time::now();
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = z;
        m.pose.orientation.w = 1.0;
        m.scale.z = scale;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = a;
        m.text = text;
        return m;
    }

    ros::NodeHandle nh_;
    ros::Subscriber cones_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber mode_sub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher marker_pub_;

    cone_car_core::ConeArray cones_;
    cone_car_core::PlanningPath path_;
    cone_car_core::ModeStatus mode_;
    cone_car_core::ControlCommand cmd_;
    cone_car_core::VehicleState state_;

    bool cones_received_ = false;
    bool path_received_ = false;
    bool mode_received_ = false;
    bool cmd_received_ = false;
    bool state_received_ = false;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_visualizer_node");
    RvizVisualizerNode node;
    ros::spin();
    return 0;
}