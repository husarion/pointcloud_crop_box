// Copyright 2025 Husarion sp. z o.o.
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

#ifndef POINTCLOUD_BOXCROP_POINTCLOUD_BOXCROP_POINTCLOUD_BOXCROP_HPP_
#define POINTCLOUD_BOXCROP_POINTCLOUD_BOXCROP_POINTCLOUD_BOXCROP_HPP_

#include <memory>
#include <string>

#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions/pcl_conversions.h>
#include <pcl_ros/pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>

#include "pointcloud_boxcrop/pointcloud_boxcrop_params.hpp"

class PointcloudBoxcrop : public rclcpp::Node {
public:
  PointcloudBoxcrop();

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::shared_ptr<pointcloud_boxcrop_params::ParamListener> param_listener_;
  pointcloud_boxcrop_params::Params params_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<vision_msgs::msg::BoundingBox3D>::SharedPtr bbox_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

#endif // POINTCLOUD_BOXCROP_POINTCLOUD_BOXCROP_POINTCLOUD_BOXCROP_HPP_
