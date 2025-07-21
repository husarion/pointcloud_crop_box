cmake_minimum_required(VERSION 3.5)
project(pointcloud2_box_filter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(vision_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

generate_parameter_library(
  pointcloud2_box_filter_params
  src/pointcloud2_box_filter_params.yaml
)

add_executable(pointcloud2_box_filter_node src/pointcloud2_box_filter.cpp  src/pointcloud2_box_filter_node.cpp)
ament_target_dependencies(pointcloud2_box_filter_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_sensor_msgs
  generate_parameter_library
  vision_msgs
)
target_link_libraries(pointcloud2_box_filter_node ${PCL_LIBRARIES}  pointcloud2_box_filter_params)
target_include_directories(pointcloud2_box_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)

ament_export_include_directories(
  include
)

install(TARGETS
  pointcloud2_box_filter_node
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
  )
ament_package()
