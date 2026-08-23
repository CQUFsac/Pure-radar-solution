#include <ros/ros.h>
#include <cone_car_core/Cone.h>
#include <cone_car_core/ConeArray.h>

class FakeConePublisher
{
public:
    FakeConePublisher()
    {
        pub_ = nh_.advertise<cone_car_core::ConeArray>("/perception/cones", 10);
        timer_ = nh_.createTimer(ros::Duration(0.1), &FakeConePublisher::timerCallback, this);
        cone_id_counter_ = 0;

        ROS_INFO("[FakeConePublisher] Initialized. Publishing at 10 Hz");
    }

private:
    void timerCallback(const ros::TimerEvent&)
    {
        cone_car_core::ConeArray msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "base_link";

        // Blue cones — left boundary
        addCone(2.0f,  1.0f, 0.0f, "blue",   "fake");
        addCone(4.0f,  1.0f, 0.0f, "blue",   "fake");
        addCone(6.0f,  1.2f, 0.0f, "blue",   "fake");
        addCone(8.0f,  1.4f, 0.0f, "blue",   "fake");
        addCone(10.0f, 1.6f, 0.0f, "blue",   "fake");

        // Yellow cones — right boundary
        addCone(2.0f,  -1.0f,  0.0f, "yellow", "fake");
        addCone(4.0f,  -1.0f,  0.0f, "yellow", "fake");
        addCone(6.0f,  -1.1f,  0.0f, "yellow", "fake");
        addCone(8.0f,  -1.3f,  0.0f, "yellow", "fake");
        addCone(10.0f, -1.5f,  0.0f, "yellow", "fake");

        msg.cones = cones_;
        pub_.publish(msg);
        cones_.clear();
    }

    void addCone(float x, float y, float z, const std::string& color, const std::string& source)
    {
        cone_car_core::Cone cone;
        cone.id = cone_id_counter_++;
        cone.x = x;
        cone.y = y;
        cone.z = z;
        cone.color = color;
        cone.confidence = 1.0f;
        cone.source = source;
        cone.age = 1;
        cones_.push_back(cone);
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Timer timer_;
    uint32_t cone_id_counter_;
    std::vector<cone_car_core::Cone> cones_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_cone_publisher");
    FakeConePublisher node;
    ros::spin();
    return 0;
}