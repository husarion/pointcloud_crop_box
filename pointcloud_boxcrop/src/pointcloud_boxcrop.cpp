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

#include <Eigen/Core>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pointcloud_boxcrop/pointcloud_boxcrop.hpp"

PointcloudBoxcrop::PointcloudBoxcrop()
    : rclcpp::Node("pointcloud_boxcrop"), tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {
  param_listener_ = std::make_shared<pointcloud_boxcrop_params::ParamListener>(
      get_node_parameters_interface());

  params_ = param_listener_->get_params();

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      params_.output_topic, 10);
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      params_.input_topic, 10,
      std::bind(&PointcloudBoxcrop::PointcloudCallback, this,
                std::placeholders::_1));
  bbox_pub_ = this->create_publisher<vision_msgs::msg::BoundingBox3D>(
      "~/filter_bounding_box", 10);

  RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void PointcloudBoxcrop::PointcloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);

  geometry_msgs::msg::TransformStamped transform_stamped =
      GetTransform(msg->header.frame_id);
  if (transform_stamped.header.frame_id.empty()) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Transform not found for frame: "
                                               << msg->header.frame_id);
    return;
  }

  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud =
      TransformCloud(cloud, transform_stamped, std::ref(transform_matrix));
  if (!transformed_cloud) {
    RCLCPP_WARN(this->get_logger(), "Cloud transformation failed.");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = Crop(transformed_cloud);
  if (!filtered_cloud) {
    RCLCPP_WARN(this->get_logger(), "Crop box filtering failed.");
    return;
  }

  Eigen::Matrix4f inverse_transform_matrix = transform_matrix.inverse();
  pcl::transformPointCloud(*filtered_cloud, *filtered_cloud,
                           inverse_transform_matrix);

  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*filtered_cloud, output_msg);
  output_msg.header = msg->header;

  pub_->publish(output_msg);

  vision_msgs::msg::BoundingBox3D bbox_msg = CreateBoundingBox();
  bbox_pub_->publish(bbox_msg);
}

geometry_msgs::msg::TransformStamped
PointcloudBoxcrop::GetTransform(const std::string &source_frame) {
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform(
        params_.target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Could not transform "
                                               << params_.target_frame << " to "
                                               << source_frame << ": "
                                               << ex.what());
  }
  return transform_stamped;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointcloudBoxcrop::TransformCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const geometry_msgs::msg::TransformStamped &transform_stamped,
    Eigen::Matrix4f &transform_matrix) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  try {
    transform_matrix.block<3, 3>(0, 0) =
        Eigen::Quaternionf(transform_stamped.transform.rotation.w,
                           transform_stamped.transform.rotation.x,
                           transform_stamped.transform.rotation.y,
                           transform_stamped.transform.rotation.z)
            .toRotationMatrix();

    transform_matrix.block<3, 1>(0, 3)
        << transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z;

    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_matrix);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Error during cloud transformation: %s",
                ex.what());
    return nullptr;
  }

  return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
PointcloudBoxcrop::Crop(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {

  pcl::CropBox<pcl::PointXYZ> crop_box_filter;
  crop_box_filter.setMin(
      Eigen::Vector4f(params_.min_x, params_.min_y, params_.min_z, 1.0));
  crop_box_filter.setMax(
      Eigen::Vector4f(params_.max_x, params_.max_y, params_.max_z, 1.0));
  crop_box_filter.setInputCloud(cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  crop_box_filter.setNegative(params_.negative);
  crop_box_filter.filter(*filtered_cloud);
  return filtered_cloud;
}

vision_msgs::msg::BoundingBox3D PointcloudBoxcrop::CreateBoundingBox() {
  vision_msgs::msg::BoundingBox3D bbox;

  bbox.center.position.x = (params_.min_x + params_.max_x) / 2.0;
  bbox.center.position.y = (params_.min_y + params_.max_y) / 2.0;
  bbox.center.position.z = (params_.min_z + params_.max_z) / 2.0;

  bbox.size.x = params_.max_x - params_.min_x;
  bbox.size.y = params_.max_y - params_.min_y;
  bbox.size.z = params_.max_z - params_.min_z;

  return bbox;
}
