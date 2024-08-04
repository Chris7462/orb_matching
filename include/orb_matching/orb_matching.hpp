#pragma once

// C++ header
#include <queue>
#include <vector>
#include <mutex>

// OpenCV header
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

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

  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;

  cv::Mat get_image_from_msg(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_match_pub_;

  std::queue<sensor_msgs::msg::Image::SharedPtr> img_left_buff_;
  std::queue<sensor_msgs::msg::Image::SharedPtr> img_right_buff_;

  std::mutex mtx_;

  std::vector<cv::KeyPoint> keypoints_left_;
  std::vector<cv::KeyPoint> keypoints_right_;
  cv::Mat descriptors_left_;
  cv::Mat descriptors_right_;

  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> descriptor_;
  cv::Ptr<cv::DescriptorMatcher> matcher_;
};

} // namespace orb_matching
