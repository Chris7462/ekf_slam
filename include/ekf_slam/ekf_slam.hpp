#pragma once

// ros header
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// geographicLib header
#include <GeographicLib/LocalCartesian.hpp>


namespace ekf_slam
{

class EkfSlam: public rclcpp::Node
{
public:
  EkfSlam();
  ~EkfSlam() = default;

  void generate_landmarks();

private:
  bool gps_init_;

  void publish_landmarkers();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

  visualization_msgs::msg::MarkerArray poles_;

  GeographicLib::LocalCartesian geo_converter_;
};

} // namespace ekf_slam
