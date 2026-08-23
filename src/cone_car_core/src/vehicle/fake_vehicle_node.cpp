#include <ros/ros.h>
#include <cone_car_core/ControlCommand.h>
#include <cone_car_core/ChassisStatus.h>

class FakeVehicleNode
{
public:
    FakeVehicleNode()
    {
        cmd_sub_ = nh_.subscribe("/control/command", 10, &FakeVehicleNode::cmdCallback, this);
        pub_ = nh_.advertise<cone_car_core::ChassisStatus>("/chassis/status", 10);

        current_speed_ = 0.0f;
        current_steering_ = 0.0f;

        ROS_INFO("[FakeVehicleNode] Initialized");
    }

private:
    void cmdCallback(const cone_car_core::ControlCommand::ConstPtr& msg)
    {
        // Simulate vehicle response (simplified)
        current_steering_ = msg->steering_angle;

        // Simple speed simulation: accelerate towards target_speed
        float speed_error = msg->target_speed - current_speed_;
        if (msg->brake > 0.1f)
        {
            // Braking
            current_speed_ = std::max(0.0f, current_speed_ - msg->brake * 0.05f);
        }
        else if (speed_error > 0.0f)
        {
            current_speed_ += msg->throttle * 0.02f;
        }
        else
        {
            current_speed_ -= 0.01f; // friction deceleration
        }
        current_speed_ = std::max(0.0f, current_speed_);

        // Publish chassis status
        cone_car_core::ChassisStatus status;
        status.header = msg->header;
        status.speed = current_speed_;
        status.steering_angle = current_steering_;
        status.throttle = msg->throttle;
        status.brake = msg->brake;
        status.gear = "DRIVE";
        status.enabled = true;
        status.emergency_stop = (msg->brake > 0.9f);

        pub_.publish(status);
    }

    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_;
    ros::Publisher pub_;

    float current_speed_;
    float current_steering_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_vehicle_node");
    FakeVehicleNode node;
    ros::spin();
    return 0;
}