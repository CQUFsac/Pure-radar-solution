#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cone_car_core/ConeArray.h>
#include <cone_car_core/VehicleState.h>
#include <cone_car_core/PlanningPath.h>
#include <cone_car_core/ModeStatus.h>
#include <cone_car_core/ControlCommand.h>
#include <cone_car_core/ChassisStatus.h>

#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>

class DataLoggerNode
{
public:
    DataLoggerNode()
    {
        // Create run directory
        std::string base_dir = ros::package::getPath("cone_car_core") + "/logs/";
        std::string run_dir = base_dir + "run_" + getTimestamp() + "/";
        mkdir_p(run_dir);

        control_csv_path_ = run_dir + "control_log.csv";
        planning_txt_path_ = run_dir + "planning_log.txt";
        mode_csv_path_ = run_dir + "mode_log.csv";
        rosbag_path_ = run_dir + "data.bag";

        // Open CSV files
        control_csv_.open(control_csv_path_);
        control_csv_ << "timestamp,mode,speed,target_speed,steering_angle,throttle,brake,lateral_error,heading_error" << std::endl;

        mode_csv_.open(mode_csv_path_);
        mode_csv_ << "timestamp,current_mode,previous_mode,switch_reason,target_speed,max_curvature,turning_radius,stable_count,emergency" << std::endl;

        planning_log_.open(planning_txt_path_);
        planning_log_ << "=== Planning Log ===" << std::endl;

        // Open rosbag for recording
        bag_.open(rosbag_path_, rosbag::bagmode::Write);

        // Subscribe to all topics
        cones_sub_ = nh_.subscribe("/perception/cones", 10, &DataLoggerNode::conesCallback, this);
        state_sub_ = nh_.subscribe("/localization/vehicle_state", 10, &DataLoggerNode::stateCallback, this);
        path_sub_ = nh_.subscribe("/planning/path", 10, &DataLoggerNode::pathCallback, this);
        mode_sub_ = nh_.subscribe("/mode/status", 10, &DataLoggerNode::modeCallback, this);
        cmd_sub_ = nh_.subscribe("/control/command", 10, &DataLoggerNode::cmdCallback, this);
        chassis_sub_ = nh_.subscribe("/chassis/status", 10, &DataLoggerNode::chassisCallback, this);

        frame_count_ = 0;

        ROS_INFO("[DataLoggerNode] Initialized");
        ROS_INFO("[DataLoggerNode] Run dir: %s", run_dir.c_str());
        ROS_INFO("[DataLoggerNode] Control CSV: %s", control_csv_path_.c_str());
        ROS_INFO("[DataLoggerNode] Mode CSV: %s", mode_csv_path_.c_str());
        ROS_INFO("[DataLoggerNode] Planning log: %s", planning_txt_path_.c_str());
        ROS_INFO("[DataLoggerNode] Rosbag: %s", rosbag_path_.c_str());
    }

    ~DataLoggerNode()
    {
        control_csv_.close();
        mode_csv_.close();
        planning_log_.close();
        bag_.close();
        ROS_INFO("[DataLoggerNode] Closed. Total frames logged: %lu", frame_count_);
    }

private:
    // ==================== Callbacks ====================
    void conesCallback(const cone_car_core::ConeArray::ConstPtr& msg)
    {
        bag_.write("/perception/cones", ros::Time::now(), *msg);
    }

    void stateCallback(const cone_car_core::VehicleState::ConstPtr& msg)
    {
        bag_.write("/localization/vehicle_state", ros::Time::now(), *msg);
        vehicle_speed_ = msg->speed;
    }

    void pathCallback(const cone_car_core::PlanningPath::ConstPtr& msg)
    {
        bag_.write("/planning/path", ros::Time::now(), *msg);

        // Log planning data
        double ts = msg->header.stamp.toSec();
        planning_log_ << std::fixed << std::setprecision(3) << ts << " | "
                      << "raw=" << msg->raw_cones.size()
                      << " filtered=" << msg->filtered_cones.size()
                      << " left=" << msg->left_boundary.size()
                      << " right=" << msg->right_boundary.size()
                      << " center=" << msg->centerline.size()
                      << " valid=" << (msg->path_valid ? "Y" : "N")
                      << " quality=" << msg->quality << std::endl;

        // Log centerline points
        if (!msg->centerline.empty())
        {
            planning_log_ << "  centerline: ";
            for (size_t i = 0; i < msg->centerline.size(); ++i)
            {
                planning_log_ << "(" << msg->centerline[i].x << "," << msg->centerline[i].y
                              << ",k=" << msg->centerline[i].curvature << ")";
                if (i < msg->centerline.size() - 1) planning_log_ << " -> ";
            }
            planning_log_ << std::endl;
        }
    }

    void modeCallback(const cone_car_core::ModeStatus::ConstPtr& msg)
    {
        bag_.write("/mode/status", ros::Time::now(), *msg);

        current_mode_ = msg->current_mode;
        double ts = msg->header.stamp.toSec();

        // CSV format
        mode_csv_ << std::fixed << std::setprecision(3) << ts
                  << "," << msg->current_mode
                  << "," << msg->previous_mode
                  << "," << msg->switch_reason
                  << "," << msg->target_speed
                  << "," << msg->max_curvature
                  << "," << msg->turning_radius
                  << "," << msg->stable_count
                  << "," << (msg->emergency ? "1" : "0")
                  << std::endl;
    }

    void cmdCallback(const cone_car_core::ControlCommand::ConstPtr& msg)
    {
        bag_.write("/control/command", ros::Time::now(), *msg);

        double ts = msg->header.stamp.toSec();

        // Control CSV format
        control_csv_ << std::fixed << std::setprecision(3) << ts
                     << "," << current_mode_
                     << "," << vehicle_speed_
                     << "," << msg->target_speed
                     << "," << msg->steering_angle
                     << "," << msg->throttle
                     << "," << msg->brake
                     << "," << msg->lateral_error
                     << "," << msg->heading_error
                     << std::endl;

        frame_count_++;
    }

    void chassisCallback(const cone_car_core::ChassisStatus::ConstPtr& msg)
    {
        bag_.write("/chassis/status", ros::Time::now(), *msg);
    }

    // ==================== Utility ====================
    std::string getTimestamp()
    {
        time_t now = time(nullptr);
        struct tm t;
        localtime_r(&now, &t);
        char buf[64];
        strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &t);
        return std::string(buf);
    }

    void mkdir_p(const std::string& path)
    {
        // Create parent directories recursively
        size_t pos = 0;
        while ((pos = path.find('/', pos + 1)) != std::string::npos)
        {
            std::string sub = path.substr(0, pos);
            mkdir(sub.c_str(), 0755);
        }
        mkdir(path.c_str(), 0755);
    }

    ros::NodeHandle nh_;

    // Subscribers
    ros::Subscriber cones_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber mode_sub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber chassis_sub_;

    // File streams
    std::ofstream control_csv_;
    std::ofstream mode_csv_;
    std::ofstream planning_log_;

    // Rosbag
    rosbag::Bag bag_;

    // Paths
    std::string control_csv_path_;
    std::string mode_csv_path_;
    std::string planning_txt_path_;
    std::string rosbag_path_;

    // State
    std::string current_mode_;
    float vehicle_speed_;
    size_t frame_count_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_logger_node");
    DataLoggerNode node;
    ros::spin();
    return 0;
}