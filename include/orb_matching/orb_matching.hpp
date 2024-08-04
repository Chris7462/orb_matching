#pragma once

// C++ header
#include <queue>
#include <mutex>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>


namespace orb_matching
{

class OrbMatching : public rclcpp::Node
{
public:
  OrbMatching();
  ~OrbMatching() = default;

private:
  void img_left_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void img_right_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_right_sub_;

//void timer_callback();
//rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr yolo_pub_;
//rclcpp::TimerBase::SharedPtr timer_;

  std::queue<sensor_msgs::msg::Image::SharedPtr> img_left_buff_;
  std::queue<sensor_msgs::msg::Image::SharedPtr> img_right_buff_;

  std::mutex mtx_;
};

} // namespace orb_matching
