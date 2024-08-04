#include "orb_matching/orb_matching.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<orb_matching::OrbMatching>());
  rclcpp::shutdown();
  return 0;
}
