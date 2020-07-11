// Copyright 2020 amsl
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TINYPOWER_ROS2__TINYPOWER_ROS2_COMPONENT_HPP_
#define TINYPOWER_ROS2__TINYPOWER_ROS2_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TINYPOWER_ROS2_EXPORT __attribute__ ((dllexport))
    #define TINYPOWER_ROS2_IMPORT __attribute__ ((dllimport))
  #else
    #define TINYPOWER_ROS2_EXPORT __declspec(dllexport)
    #define TINYPOWER_ROS2_IMPORT __declspec(dllimport)
  #endif
  #ifdef TINYPOWER_ROS2_BUILDING_DLL
    #define TINYPOWER_ROS2_PUBLIC TINYPOWER_ROS2_EXPORT
  #else
    #define TINYPOWER_ROS2_PUBLIC TINYPOWER_ROS2_IMPORT
  #endif
  #define TINYPOWER_ROS2_PUBLIC_TYPE TINYPOWER_ROS2_PUBLIC
  #define TINYPOWER_ROS2_LOCAL
#else
  #define TINYPOWER_ROS2_EXPORT __attribute__ ((visibility("default")))
  #define TINYPOWER_ROS2_IMPORT
  #if __GNUC__ >= 4
    #define TINYPOWER_ROS2_PUBLIC __attribute__ ((visibility("default")))
    #define TINYPOWER_ROS2_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TINYPOWER_ROS2_PUBLIC
    #define TINYPOWER_ROS2_LOCAL
  #endif
  #define TINYPOWER_ROS2_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

namespace tinypower_ros2
{
class TinypowerROS2Component : public rclcpp::Node
{
public:
  TINYPOWER_ROS2_PUBLIC
  explicit TinypowerROS2Component(const rclcpp::NodeOptions & options);
  ~TinypowerROS2Component(void);

private:
  void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timer_callback(void);
  void open_port(const std::string & port_name);
  void close_port(void);
  void initialize_port(void);
  void send_velocity(float v, float omega);
  bool request_odom(void);
  void receive_data(void);
  int write_data(const std::string & data);
  bool request_data(
    const std::string & command, const std::string & begin, const std::string & end,
    std::string & buffer);
  std::vector<std::string> split(const std::string & str, const std::string & delimiter);
  void update_odometry(double dt);
  geometry_msgs::msg::Quaternion get_quaternion_msg_from_yaw(const double yaw);

  std::string port_name_;
  int baud_rate_;
  int buffer_size_;
  double hz_;
  /// time to try reconnection [s]
  double reconnection_interval_;
  /// timeout for poll [ms]
  int poll_timeout_;
  std::string robot_frame_id_;
  std::string odom_frame_id_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int fd_;
  std::shared_ptr<std::thread> receiving_thread_;
  std::mutex mtx_;
  double velocity_;
  double yawrate_;
  rclcpp::Clock clock_;
  rclcpp::Time last_time_;
  bool is_first_timer_callback_;
  nav_msgs::msg::Odometry odom_;
};

}  // namespace tinypower_ros2
#endif  // TINYPOWER_ROS2__TINYPOWER_ROS2_COMPONENT_HPP_
