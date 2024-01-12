#include "ekf_slam/ekf_slam.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ekf_slam::EkfSlam>());
  rclcpp::shutdown();

  return 0;
}
