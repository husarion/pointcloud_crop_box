#include "rclcpp/rclcpp.hpp"


#include "pointcloud2_box_filter/pointcloud2_box_filter.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloud2BoxFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
