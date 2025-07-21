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

#include <iostream>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include "pointcloud_boxcrop/pointcloud_boxcrop.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto pointcloud_boxcrop = std::make_shared<PointcloudBoxcrop>();

  try {
    rclcpp::spin(pointcloud_boxcrop);
  } catch (const std::runtime_error &e) {
    std::cerr << "[" << pointcloud_boxcrop->get_name()
              << "] Caught exception: " << e.what() << std::endl;
  }

  std::cout << "[" << pointcloud_boxcrop->get_name() << "] Shutting down"
            << std::endl;
  rclcpp::shutdown();
  return 0;
}
