#ifndef POINTCLOUD2_BOX_FILTER_HPP
#define POINTCLOUD2_BOX_FILTER_HPP

#include "pointcloud2_box_filter/pointcloud2_box_filter_params.hpp"
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions/pcl_conversions.h>
#include <pcl_ros/pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class PointCloud2BoxFilter : public rclcpp::Node {
public:
  PointCloud2BoxFilter();

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::shared_ptr<pointcloud2_box_filter_params::ParamListener> param_listener_;
  pointcloud2_box_filter_params::Params params_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<vision_msgs::msg::BoundingBox3D>::SharedPtr bbox_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

#endif // POINTCLOUD2_BOX_FILTER_HPP
