#include <ros/ros.h>
#include <cone_car_core/PlanningPath.h>
#include <cone_car_core/VehicleState.h>
#include <cone_car_core/ModeStatus.h>
#include <cmath>
#include <string>

class ModeManagerNode
{
public:
    ModeManagerNode()
    {
        path_sub_ = nh_.subscribe("/planning/path", 10, &ModeManagerNode::pathCallback, this);
        state_sub_ = nh_.subscribe("/localization/vehicle_state", 10, &ModeManagerNode::stateCallback, this);
        pub_ = nh_.advertise<cone_car_core::ModeStatus>("/mode/status", 10);

        nh_.param<float>("mode/sharp_curve_curvature_threshold", curvature_threshold_, 0.25f);
        nh_.param<int>("mode/recovery_stable_frames", recovery_stable_frames_, 10);
        nh_.param<float>("speed/normal_speed", normal_speed_, 1.5f);
        nh_.param<float>("speed/sharp_curve_speed", sharp_curve_speed_, 0.8f);
        nh_.param<float>("speed/recovery_speed", recovery_speed_, 1.0f);
        nh_.param<float>("speed/emergency_speed", emergency_speed_, 0.0f);

        current_mode_ = "NORMAL";
        stable_count_ = 0;
        max_curvature_ = 0.0f;

        ROS_INFO("[ModeManagerNode] Initialized. curvature_threshold=%.2f, recovery_frames=%d",
                 curvature_threshold_, recovery_stable_frames_);
    }

private:
    void stateCallback(const cone_car_core::VehicleState::ConstPtr& msg)
    {
        vehicle_speed_ = msg->speed;
    }

    void pathCallback(const cone_car_core::PlanningPath::ConstPtr& msg)
    {
        std::string previous_mode = current_mode_;
        std::string switch_reason = "";
        bool emergency = false;

        // Find max curvature in centerline
        max_curvature_ = 0.0f;
        float turning_radius = 1e6f;
        for (const auto& pt : msg->centerline)
        {
            float abs_k = std::abs(pt.curvature);
            if (abs_k > max_curvature_)
            {
                max_curvature_ = abs_k;
            }
        }
        if (max_curvature_ > 1e-4f)
        {
            turning_radius = 1.0f / max_curvature_;
        }

        // Mode decision logic
        if (!msg->path_valid)
        {
            // Path invalid -> EMERGENCY
            current_mode_ = "EMERGENCY";
            switch_reason = "path_invalid";
            emergency = true;
            stable_count_ = 0;
        }
        else if (max_curvature_ > curvature_threshold_)
        {
            // Curvature too high -> SHARP_CURVE
            current_mode_ = "SHARP_CURVE";
            switch_reason = "curvature=" + std::to_string(max_curvature_) + " > threshold";
            stable_count_ = 0;
        }
        else if (current_mode_ == "SHARP_CURVE" || current_mode_ == "EMERGENCY")
        {
            // Exiting sharp curve or emergency -> RECOVERY
            current_mode_ = "RECOVERY";
            switch_reason = "exiting_" + previous_mode;
            stable_count_++;
        }
        else if (current_mode_ == "RECOVERY")
        {
            stable_count_++;
            if (stable_count_ >= (uint32_t)recovery_stable_frames_)
            {
                current_mode_ = "NORMAL";
                switch_reason = "recovery_complete";
                stable_count_ = 0;
            }
        }
        else
        {
            current_mode_ = "NORMAL";
            stable_count_++;
        }

        // Determine target speed
        float target_speed = normal_speed_;
        if (current_mode_ == "SHARP_CURVE")
            target_speed = sharp_curve_speed_;
        else if (current_mode_ == "RECOVERY")
            target_speed = recovery_speed_;
        else if (current_mode_ == "EMERGENCY")
            target_speed = emergency_speed_;

        // Publish mode status
        cone_car_core::ModeStatus status;
        status.header = msg->header;
        status.current_mode = current_mode_;
        status.previous_mode = previous_mode;
        status.switch_reason = switch_reason;
        status.target_speed = target_speed;
        status.max_curvature = max_curvature_;
        status.turning_radius = turning_radius;
        status.stable_count = stable_count_;
        status.emergency = emergency;

        pub_.publish(status);
    }

    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher pub_;

    float curvature_threshold_;
    int recovery_stable_frames_;
    float normal_speed_, sharp_curve_speed_, recovery_speed_, emergency_speed_;
    float vehicle_speed_;

    std::string current_mode_;
    uint32_t stable_count_;
    float max_curvature_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mode_manager_node");
    ModeManagerNode node;
    ros::spin();
    return 0;
}