#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <serial/serial.h>

#include <algorithm>
#include <boost/array.hpp>
#include <cmath>
#include <cstdint>
#include <string>

extern "C"
{
#include "imu_data_decode.h"
#include "packet.h"
}

namespace
{

constexpr double kGravity = 9.80665;
constexpr double kDegreesToRadians = 0.017453292519943295;

void setDiagonal(
  boost::array<double, 9>& covariance,
  const double value)
{
  covariance.assign(0.0);
  covariance[0] = value;
  covariance[4] = value;
  covariance[8] = value;
}

bool fillImuMessage(
  const receive_imusol_packet_t& data,
  const std::string& frame_id,
  const double orientation_variance,
  const double angular_velocity_variance,
  const double linear_acceleration_variance,
  sensor_msgs::Imu& message)
{
  message = sensor_msgs::Imu();
  message.header.stamp = ros::Time::now();
  message.header.frame_id = frame_id;

  if ((bitmap & BIT_VALID_QUAT) != 0U)
  {
    const double norm = std::sqrt(
      data.quat[0] * data.quat[0] +
      data.quat[1] * data.quat[1] +
      data.quat[2] * data.quat[2] +
      data.quat[3] * data.quat[3]);
    if (norm < 1.0e-6)
    {
      message.orientation_covariance[0] = -1.0;
    }
    else
    {
      message.orientation.w = data.quat[0] / norm;
      message.orientation.x = data.quat[1] / norm;
      message.orientation.y = data.quat[2] / norm;
      message.orientation.z = data.quat[3] / norm;
      setDiagonal(message.orientation_covariance, orientation_variance);
    }
  }
  else
  {
    message.orientation_covariance[0] = -1.0;
  }

  if ((bitmap & BIT_VALID_GYR) != 0U)
  {
    message.angular_velocity.x = data.gyr[0] * kDegreesToRadians;
    message.angular_velocity.y = data.gyr[1] * kDegreesToRadians;
    message.angular_velocity.z = data.gyr[2] * kDegreesToRadians;
    setDiagonal(
      message.angular_velocity_covariance,
      angular_velocity_variance);
  }
  else
  {
    message.angular_velocity_covariance[0] = -1.0;
  }

  if ((bitmap & BIT_VALID_ACC) != 0U)
  {
    message.linear_acceleration.x = data.acc[0] * kGravity;
    message.linear_acceleration.y = data.acc[1] * kGravity;
    message.linear_acceleration.z = data.acc[2] * kGravity;
    setDiagonal(
      message.linear_acceleration_covariance,
      linear_acceleration_variance);
  }
  else
  {
    message.linear_acceleration_covariance[0] = -1.0;
  }

  return (bitmap & (BIT_VALID_GYR | BIT_VALID_ACC | BIT_VALID_QUAT)) != 0U;
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "serial_imu");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string port;
  std::string frame_id;
  std::string output_topic;
  int baud_rate = 115200;
  int timeout_ms = 100;
  int queue_size = 50;
  double loop_rate = 500.0;
  double orientation_variance = 0.01;
  double angular_velocity_variance = 0.0025;
  double linear_acceleration_variance = 0.04;

  private_nh.param<std::string>("port", port, "/dev/imu");
  private_nh.param("baud_rate", baud_rate, 115200);
  private_nh.param("timeout_ms", timeout_ms, 100);
  private_nh.param<std::string>("frame_id", frame_id, "imu_link");
  private_nh.param<std::string>("output_topic", output_topic, "/imu/data");
  private_nh.param("queue_size", queue_size, 50);
  private_nh.param("loop_rate", loop_rate, 500.0);
  private_nh.param(
    "orientation_variance", orientation_variance, 0.01);
  private_nh.param(
    "angular_velocity_variance",
    angular_velocity_variance,
    0.0025);
  private_nh.param(
    "linear_acceleration_variance",
    linear_acceleration_variance,
    0.04);

  ros::Publisher publisher = nh.advertise<sensor_msgs::Imu>(
    output_topic, std::max(1, queue_size));

  serial::Serial serial_port;
  serial_port.setPort(port);
  serial_port.setBaudrate(static_cast<std::uint32_t>(
    std::max(1, baud_rate)));
  serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(
    static_cast<std::uint32_t>(std::max(1, timeout_ms)));
  serial_port.setTimeout(serial_timeout);

  imu_data_decode_init();

  try
  {
    serial_port.open();
  }
  catch (const serial::IOException& exception)
  {
    ROS_FATAL(
      "Cannot open IMU port %s: %s",
      port.c_str(),
      exception.what());
    return 1;
  }
  if (!serial_port.isOpen())
  {
    ROS_FATAL("IMU serial port did not open: %s", port.c_str());
    return 1;
  }

  ROS_INFO(
    "serial_imu opened %s at %d baud, publishing %s in %s",
    port.c_str(),
    baud_rate,
    output_topic.c_str(),
    frame_id.c_str());

  ros::Rate rate(std::max(1.0, loop_rate));
  std::uint8_t buffer[2048];
  while (ros::ok())
  {
    const std::size_t available = serial_port.available();
    if (available > 0U)
    {
      const std::size_t count = serial_port.read(
        buffer, std::min<std::size_t>(available, sizeof(buffer)));
      for (std::size_t index = 0U; index < count; ++index)
      {
        packet_decode(buffer[index]);
      }

      sensor_msgs::Imu message;
      if (receive_gwsol.tag == KItemGWSOL)
      {
        const std::size_t device_count = std::min<std::size_t>(
          receive_gwsol.n, MAX_LENGTH);
        for (std::size_t index = 0U; index < device_count; ++index)
        {
          if (fillImuMessage(
                receive_gwsol.receive_imusol[index],
                frame_id,
                orientation_variance,
                angular_velocity_variance,
                linear_acceleration_variance,
                message))
          {
            publisher.publish(message);
          }
        }
      }
      else if (fillImuMessage(
                 receive_imusol,
                 frame_id,
                 orientation_variance,
                 angular_velocity_variance,
                 linear_acceleration_variance,
                 message))
      {
        publisher.publish(message);
      }
    }
    ros::spinOnce();
    rate.sleep();
  }

  serial_port.close();
  return 0;
}
