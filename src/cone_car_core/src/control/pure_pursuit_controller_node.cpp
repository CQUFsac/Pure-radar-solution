#include <ros/ros.h>
#include <cone_car_core/PlanningPath.h>
#include <cone_car_core/VehicleState.h>
#include <cone_car_core/ModeStatus.h>
#include <cone_car_core/ControlCommand.h>
#include <cmath>
#include <algorithm>

class PurePursuitController
{
public:
    PurePursuitController()
    {
        path_sub_ = nh_.subscribe("/planning/path", 10, &PurePursuitController::pathCallback, this);
        state_sub_ = nh_.subscribe("/localization/vehicle_state", 10, &PurePursuitController::stateCallback, this);
        mode_sub_ = nh_.subscribe("/mode/status", 10, &PurePursuitController::modeCallback, this);
        pub_ = nh_.advertise<cone_car_core::ControlCommand>("/control/command", 10);

        nh_.param<float>("control/lookahead_distance", lookahead_base_, 1.5f);
        nh_.param<float>("control/max_steering_angle", max_steering_, 0.5f);
        nh_.param<float>("control/max_steering_rate", max_steering_rate_, 0.1f);
        nh_.param<float>("control/throttle_kp", throttle_kp_, 0.5f);
        nh_.param<float>("control/brake_value_emergency", brake_emergency_, 1.0f);

        has_path_ = false;
        has_state_ = false;
        has_mode_ = false;
        prev_steering_ = 0.0f;

        ROS_INFO("[PurePursuitController] Initialized");
    }

private:
    void stateCallback(const cone_car_core::VehicleState::ConstPtr& msg)
    {
        vehicle_state_ = *msg;
        has_state_ = true;
    }

    void modeCallback(const cone_car_core::ModeStatus::ConstPtr& msg)
    {
        mode_status_ = *msg;
        has_mode_ = true;
    }

    void pathCallback(const cone_car_core::PlanningPath::ConstPtr& msg)
    {
        path_ = *msg;
        has_path_ = true;

        if (!has_state_ || !has_mode_) return;

        cone_car_core::ControlCommand cmd;
        cmd.header = msg->header;

        // Emergency: full brake, zero throttle
        if (mode_status_.current_mode == "EMERGENCY")
        {
            cmd.steering_angle = 0.0f;
            cmd.target_speed = 0.0f;
            cmd.throttle = 0.0f;
            cmd.brake = brake_emergency_;
            cmd.lookahead_distance = lookahead_base_;
            cmd.target_point_x = 0.0f;
            cmd.target_point_y = 0.0f;
            cmd.lateral_error = 0.0f;
            cmd.heading_error = 0.0f;
            pub_.publish(cmd);
            return;
        }

        // Pure Pursuit on centerline
        const auto& cl = path_.centerline;
        if (cl.empty() || !path_.path_valid)
        {
            cmd.steering_angle = 0.0f;
            cmd.target_speed = 0.0f;
            cmd.throttle = 0.0f;
            cmd.brake = 0.5f;
            pub_.publish(cmd);
            return;
        }

        // Adjust lookahead based on mode
        float lookahead = lookahead_base_;
        if (mode_status_.current_mode == "SHARP_CURVE")
            lookahead = lookahead_base_ * 0.6f;
        else if (mode_status_.current_mode == "RECOVERY")
            lookahead = lookahead_base_ * 0.8f;

        // Find the target point (closest to lookahead distance ahead)
        float best_dist = 1e9f;
        size_t target_idx = 0;
        for (size_t i = 0; i < cl.size(); ++i)
        {
            float dx = cl[i].x - vehicle_state_.x;
            float dy = cl[i].y - vehicle_state_.y;
            float dist = std::sqrt(dx * dx + dy * dy);
            float diff = std::abs(dist - lookahead);
            if (diff < best_dist)
            {
                best_dist = diff;
                target_idx = i;
            }
        }

        float tx = cl[target_idx].x;
        float ty = cl[target_idx].y;

        // Pure Pursuit steering calculation
        float dx = tx - vehicle_state_.x;
        float dy = ty - vehicle_state_.y;
        float ld = std::sqrt(dx * dx + dy * dy);

        // Transform target to vehicle frame
        float cos_yaw = std::cos(vehicle_state_.yaw);
        float sin_yaw = std::sin(vehicle_state_.yaw);
        float local_x = cos_yaw * dx + sin_yaw * dy;
        float local_y = -sin_yaw * dx + cos_yaw * dy;

        // Steering angle = atan2(2 * L * local_y, ld^2)
        // L = wheelbase, approximated as ld for simple version
        float steering = 0.0f;
        if (ld > 1e-4f)
        {
            steering = std::atan2(2.0f * local_y, ld);
        }

        // Clamp steering
        steering = std::max(-max_steering_, std::min(max_steering_, steering));

        // Rate limit steering
        float d_steering = steering - prev_steering_;
        d_steering = std::max(-max_steering_rate_, std::min(max_steering_rate_, d_steering));
        steering = prev_steering_ + d_steering;
        prev_steering_ = steering;

        // Lateral error (perpendicular distance to nearest centerline point)
        float lateral_error = local_y;
        float heading_error = std::atan2(local_y, local_x);

        // Throttle / brake based on target speed
        float target_speed = mode_status_.target_speed;
        float speed_error = target_speed - vehicle_state_.speed;
        float throttle = 0.0f;
        float brake = 0.0f;
        if (speed_error > 0.0f)
        {
            throttle = std::min(1.0f, throttle_kp_ * speed_error);
        }
        else
        {
            brake = std::min(1.0f, throttle_kp_ * (-speed_error));
        }

        cmd.steering_angle = steering;
        cmd.target_speed = target_speed;
        cmd.throttle = throttle;
        cmd.brake = brake;
        cmd.lookahead_distance = lookahead;
        cmd.target_point_x = tx;
        cmd.target_point_y = ty;
        cmd.lateral_error = lateral_error;
        cmd.heading_error = heading_error;

        pub_.publish(cmd);
    }

    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber mode_sub_;
    ros::Publisher pub_;

    cone_car_core::PlanningPath path_;
    cone_car_core::VehicleState vehicle_state_;
    cone_car_core::ModeStatus mode_status_;

    bool has_path_, has_state_, has_mode_;
    float lookahead_base_, max_steering_, max_steering_rate_;
    float throttle_kp_, brake_emergency_;
    float prev_steering_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pure_pursuit_controller_node");
    PurePursuitController node;
    ros::spin();
    return 0;
}