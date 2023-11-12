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

#include <memory>
#include <string>
#include <vector>

#include "tinypower_ros2/tinypower_ros2_component.hpp"

namespace tinypower_ros2
{
TinypowerROS2Component::TinypowerROS2Component(const rclcpp::NodeOptions & options)
: Node("tinypower_ros2", options),
  fd_(-1),
  velocity_(0.0),
  yawrate_(0.0),
  clock_(RCL_ROS_TIME),
  is_first_timer_callback_(true)
{
  declare_parameter<std::string>("port_name", "/dev/tiny");
  get_parameter("port_name", port_name_);
  declare_parameter<int>("baud_rate", 57600);
  get_parameter("baud_rate", baud_rate_);
  declare_parameter<int>("buffer_size", 4096);
  get_parameter("buffer_size", buffer_size_);
  declare_parameter<double>("reconnection_interval", 0.5);
  get_parameter("reconnection_interval", reconnection_interval_);
  declare_parameter<double>("hz", 40.0);
  get_parameter("hz", hz_);
  declare_parameter<int>("poll_timeout", 1000);
  get_parameter("poll_timeout", poll_timeout_);
  declare_parameter<std::string>("robot_frame_id", "base_link");
  get_parameter("robot_frame_id", robot_frame_id_);
  declare_parameter<std::string>("odom_frame_id", "odom");
  get_parameter("odom_frame_id", odom_frame_id_);
  declare_parameter<bool>("enable_tf", true);
  get_parameter("enable_tf", enable_tf_);

  odom_.pose.pose.orientation = get_quaternion_msg_from_yaw(0);
  tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

  timer_ =
    create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1.0 / hz_ * 1e3)),
    std::bind(&TinypowerROS2Component::timer_callback, this));
  vel_sub_ =
    create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1,
    std::bind(&TinypowerROS2Component::velocity_callback, this, std::placeholders::_1));

  open_port(port_name_);
  initialize_port();
  // receiving_thread_ = std::make_shared<std::thread>(&TinypowerROS2Component::receive_data, this);
}

TinypowerROS2Component::~TinypowerROS2Component(void)
{
  close_port();
  receiving_thread_->join();
}

void TinypowerROS2Component::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "velocity_callback");
  send_velocity(msg->linear.x, msg->angular.z);
}

void TinypowerROS2Component::timer_callback(void)
{
  RCLCPP_INFO(get_logger(), "timer_callback");
  if (request_odom()) {
    rclcpp::Time now = clock_.now();
    if (!is_first_timer_callback_) {
      update_odometry((now - last_time_).nanoseconds() * 1e-9);
      odom_.header.stamp = now;
      odom_pub_->publish(odom_);
      if (enable_tf_) {
        publish_odom_tf();
      }
    } else {
      is_first_timer_callback_ = false;
    }
    last_time_ = now;
  }
}

void TinypowerROS2Component::open_port(const std::string & port_name)
{
  rclcpp::WallRate rate(1.0 / reconnection_interval_);
  while (fd_ < 0 && rclcpp::ok()) {
    fd_ = open(port_name.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
    if (fd_ >= 0) {
      RCLCPP_INFO_STREAM(get_logger(), "Successfully connected to the device: " + port_name);
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to open port: " + port_name);
    }
    rate.sleep();
  }
}

void TinypowerROS2Component::close_port(void)
{
  if (fd_ >= 0) {
    close(fd_);
  }
}

void TinypowerROS2Component::initialize_port(void)
{
  struct termios newtio;
  tcgetattr(fd_, &newtio);
  memset(&newtio.c_cc, 0, sizeof(newtio.c_cc));
  newtio.c_cflag = CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  cfsetspeed(&newtio, baud_rate_);
  tcflush(fd_, TCIFLUSH);
  const int result = tcsetattr(fd_, TCSANOW, &newtio);
  if (result < 0) {
    RCLCPP_FATAL(get_logger(), "Failed to set serial port attributes");
    close_port();
    exit(-1);
  }
  ioctl(fd_, TCSETS, &newtio);
}

void TinypowerROS2Component::send_velocity(float v, float omega)
{
  std::stringstream ss;
  ss << "VCX" << v << "\n\r" << "VCR" << omega << "\n\r";
  const std::string data = ss.str();
  RCLCPP_INFO_STREAM(get_logger(), data);
  if (write_data(data) <= 0) {
    RCLCPP_ERROR(get_logger(), "Failed to send data");
  }
}

bool TinypowerROS2Component::request_odom(void)
{
  const std::string command = "MVV";
  const std::string begin = std::to_string(command[1]);
  const std::string end = "\n";
  std::string data;

  // received data expected to be "$MVV: <vel>, <yawrate>, <icount1>, <icount2>\n"
  const bool result = request_data(command + "\r\n", begin, end, data);
  if (result) {
    std::vector<std::string> splitted_data = split(data, "\n\r>$, :");
    for (auto it = splitted_data.begin(); it != splitted_data.end(); ++it) {
      if (*it == command) {
        ++it;
        velocity_ = std::stod(*it);
        ++it;
        yawrate_ = std::stod(*it);
        return true;
      }
    }
  }
  return false;
}

void TinypowerROS2Component::receive_data(void)
{
  while (rclcpp::ok()) {
    RCLCPP_INFO(get_logger(), "loop");
    // request_odom();
  }
}

int TinypowerROS2Component::write_data(const std::string & data)
{
  std::lock_guard<std::mutex> lock(mtx_);
  return ::write(fd_, data.c_str(), strlen(data.c_str()));
}

bool TinypowerROS2Component::request_data(
  const std::string & command, const std::string & begin,
  const std::string & end, std::string & buffer)
{
  bool is_begin_received = false;

  struct pollfd fds[1];
  fds[0].fd = fd_;
  fds[0].events = POLLIN;

  buffer.clear();

  if (write_data(command)) {
    while (buffer.size() < static_cast<unsigned int>(buffer_size_)) {
      if (poll(fds, 1, poll_timeout_) == 0) {
        RCLCPP_ERROR(get_logger(), "Poll reached timeout");
        return false;
      }
      if (fds[0].revents & POLLERR) {
        RCLCPP_ERROR(get_logger(), "Error on socket");
        return false;
      }
      // use magic number here to avoid warning
      char buffer_c[256];
      int size = 0;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        size = ::read(fd_, buffer_c, 256);
      }
      if (size >= 0) {
        buffer.append(buffer_c, size);
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to read");
        return false;
      }

      if (!is_begin_received) {
        const int pos = buffer.find_first_of(begin);
        if (pos > 0) {
          buffer.erase(0, pos);
          is_begin_received = true;
        }
      } else {
        const int pos = buffer.find_first_of(end);
        if (pos > 0) {
          buffer.erase(pos + 1, buffer.size() - (pos + 1));
          return true;
        }
      }
    }
  }
  RCLCPP_ERROR(get_logger(), "Failed to receive expected data");
  return false;
}

std::vector<std::string> TinypowerROS2Component::split(
  const std::string & str,
  const std::string & delimiter)
{
  std::vector<std::string> str_vector;
  std::string data = str;
  std::string::size_type pos = str.npos;
  while ((pos = data.find_first_of(delimiter)) != str.npos) {
    if (pos > 0) {
      str_vector.push_back(data.substr(0, pos));
    }
    data = data.substr(pos + 1);
  }
  return str_vector;
}

void TinypowerROS2Component::update_odometry(double dt)
{
  odom_.header.frame_id = odom_frame_id_;
  odom_.child_frame_id = robot_frame_id_;
  const double yaw = tf2::getYaw(odom_.pose.pose.orientation);
  const double dx = velocity_ * dt;
  const double dyaw = yawrate_ * dt;
  odom_.pose.pose.position.x += dx * cos(yaw);
  odom_.pose.pose.position.y += dx * sin(yaw);
  odom_.pose.pose.orientation = get_quaternion_msg_from_yaw(yaw + dyaw);
  odom_.twist.twist.linear.x = velocity_;
  odom_.twist.twist.angular.z = yawrate_;
}

geometry_msgs::msg::Quaternion TinypowerROS2Component::get_quaternion_msg_from_yaw(const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

void TinypowerROS2Component::publish_odom_tf(void)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header = odom_.header;
  transform.child_frame_id = odom_.child_frame_id;
  transform.transform.translation.x = odom_.pose.pose.position.x;
  transform.transform.translation.y = odom_.pose.pose.position.y;
  transform.transform.translation.z = odom_.pose.pose.position.z;
  transform.transform.rotation = odom_.pose.pose.orientation;
  tfb_->sendTransform(transform);
}

}  // namespace tinypower_ros2
