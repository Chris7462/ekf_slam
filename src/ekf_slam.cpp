// c++ header
#include <chrono>

// ros header
#include <visualization_msgs/msg/marker.hpp>

// local header
#include "ekf_slam/ekf_slam.hpp"


namespace ekf_slam
{

using namespace std::chrono_literals;

EkfSlam::EkfSlam()
: Node("ekf_slam_node"), gps_init_{false}
{
  std::vector<double> init_p = declare_parameter("init_p", std::vector<double>());

  const int pole_number = 12;
  std::vector<std::vector<double>> poles(pole_number, std::vector<double>());
  for (int i = 0; i < pole_number; ++i) {
    poles[i] = declare_parameter("pole" + std::to_string(i), std::vector<double>());
  }

  rclcpp::QoS qos(10);

  timer_ = this->create_wall_timer(500ms, std::bind(&EkfSlam::publish_landmarkers, this));
  marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("landmarks", qos);

  generate_landmarks();
  geo_converter_.Reset(init_p[0], init_p[1], init_p[2]);

  for (auto & marker : poles_.markers) {
    geo_converter_.Forward(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z,
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
    RCLCPP_INFO(get_logger(), "Marker value = %f, %f, %f",
      marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
  }
}

void EkfSlam::publish_landmarkers()
{
  rclcpp::Time current_time = rclcpp::Node::now();
  marker_array_pub_->publish(poles_);
}

void EkfSlam::generate_landmarks()
{
  visualization_msgs::msg::Marker pole1;
  pole1.header.stamp = rclcpp::Node::now();
  pole1.header.frame_id = "map";
  pole1.ns = "marker";
  pole1.id = 1;
  pole1.type = visualization_msgs::msg::Marker::CYLINDER;
  pole1.action = visualization_msgs::msg::Marker::ADD;
  pole1.pose.position.x = 49.00877;
  pole1.pose.position.y = 8.39774;
  pole1.pose.position.z = 112.58;
  pole1.scale.x = 1.0;
  pole1.scale.y = 1.0;
  pole1.scale.z = 3.0;
  pole1.color.a = 1.0;
  pole1.color.r = 1.0;
  poles_.markers.push_back(pole1);

  visualization_msgs::msg::Marker pole2;
  pole2.header.stamp = rclcpp::Node::now();
  pole2.header.frame_id = "map";
  pole1.ns = "marker";
  pole2.id = 2;
  pole2.type = visualization_msgs::msg::Marker::CYLINDER;
  pole2.action = visualization_msgs::msg::Marker::ADD;
  pole2.pose.position.x = 49.00883;
  pole2.pose.position.y = 8.39754;
  pole2.pose.position.z = 112.51;
  pole2.scale.x = 1.0;
  pole2.scale.y = 1.0;
  pole2.scale.z = 3.0;
  pole2.color.a = 1.0;
  pole2.color.r = 1.0;
  poles_.markers.push_back(pole2);
}

}  // namespace ekf_slam
