// C++ header
#include <chrono>

// ROS header
#include <cv_bridge/cv_bridge.h>

// local header
#include "orb_matching/orb_matching.hpp"


namespace orb_matching
{
using namespace std::chrono_literals;

OrbMatching::OrbMatching()
: Node("orb_matching_node")
{
  rclcpp::QoS qos(10);
  img_left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "kitti/camera/color/left/image_raw", qos, std::bind(
      &OrbMatching::img_left_callback, this, std::placeholders::_1));

  img_right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "kitti/camera/color/right/image_raw", qos, std::bind(
      &OrbMatching::img_right_callback, this, std::placeholders::_1));

  img_match_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "orb_matching/img_match", qos);

  timer_ = this->create_wall_timer(
    25ms, std::bind(&OrbMatching::timer_callback, this));

  detector_ = cv::ORB::create();
  descriptor_ = cv::ORB::create();
  matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

void OrbMatching::img_left_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  img_left_buff_.push(msg);
}

void OrbMatching::img_right_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  img_right_buff_.push(msg);
}

cv::Mat OrbMatching::get_image_from_msg(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cv_bridge::CvImageConstPtr ptr;
  try {
    ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exeception: %s", e.what());
  }
  return ptr->image;
}

void OrbMatching::timer_callback()
{
  cv::Mat img_left, img_right;
  if (!img_left_buff_.empty() && !img_right_buff_.empty()) {
    mtx_.lock();
    double left_time = rclcpp::Time(img_left_buff_.front()->header.stamp).seconds();
    double right_time = rclcpp::Time(img_right_buff_.front()->header.stamp).seconds();

    // sync tolerance
    if (left_time < right_time - 0.015) {
      img_left_buff_.pop();
      RCLCPP_INFO_STREAM(get_logger(), "Throw left image --- Sync error: " << (left_time - right_time));
      mtx_.unlock();
    } else if (left_time > right_time + 0.015) {
      img_right_buff_.pop();
      RCLCPP_INFO_STREAM(get_logger(), "Throw right image --- Sync error: " << (left_time - right_time));
      mtx_.unlock();
    } else {
      img_left = get_image_from_msg(img_left_buff_.front());
      img_right = get_image_from_msg(img_right_buff_.front());
      img_left_buff_.pop();
      img_right_buff_.pop();
      mtx_.unlock();

      if (!img_left.empty() && !img_right.empty()) {
        // detect Oriented FAST
        rclcpp::Time start = rclcpp::Node::now();
        detector_->detect(img_left, keypoints_left_);
        detector_->detect(img_right, keypoints_right_);

        // compute BRIEF descriptor
        descriptor_->compute(img_left, keypoints_left_, descriptors_left_);
        descriptor_->compute(img_right, keypoints_right_, descriptors_right_);
        rclcpp::Time end = rclcpp::Node::now();
        RCLCPP_INFO(get_logger(), "Extract ORB cost = %f seconds.", (end-start).seconds());

        // use Hamming distance to match the features
        std::vector<cv::DMatch> matches;
        start = rclcpp::Node::now();
        matcher_->match(descriptors_left_, descriptors_right_, matches);
        end = rclcpp::Node::now();
        RCLCPP_INFO(get_logger(), "Match ORB cost = %f seconds.", (end-start).seconds());

        // sort and remove the outliers
        // min and max distance
        auto min_max = std::minmax_element(matches.begin(), matches.end(),
          [](const cv::DMatch &m_left, const cv::DMatch &m_right) {
            return m_left.distance < m_right.distance;
          });
        double min_dist = min_max.first->distance;
        // double max_dist = min_max.second->distance;

        // remove the bad matching
        std::vector<cv::DMatch> good_matches;
        for (int i = 0; i < descriptors_left_.rows; ++i) {
          if (matches[i].distance <= std::max(2 * min_dist, 30.0)) {
            good_matches.push_back(matches[i]);
          }
        }

        // publish the results
        cv::Mat img_goodmatch;
        cv::drawMatches(img_left, keypoints_left_, img_right, keypoints_right_, good_matches, img_goodmatch);
        cv_bridge::CvImage cv_img_good;
        cv_img_good.image = img_goodmatch;
        cv_img_good.encoding = "bgr8";

        auto img_good_msg = cv_img_good.toImageMsg();
        img_match_pub_->publish(*img_good_msg);
      }
    }
  }
}

} // namespace yolo_object_detection
