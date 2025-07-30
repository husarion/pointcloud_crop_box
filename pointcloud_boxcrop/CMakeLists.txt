cmake_minimum_required(VERSION 3.11)
project(pointcloud_boxcrop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    generate_parameter_library
    pcl_conversions
    pcl_ros
    rclcpp
    sensor_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    tf2_sensor_msgs
    vision_msgs)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

find_package(PCL REQUIRED COMPONENTS filters)

include_directories(include ${PCL_INCLUDE_DIRS})

generate_parameter_library(pointcloud_boxcrop_params
                           src/pointcloud_boxcrop_params.yaml)

add_executable(pointcloud_boxcrop_node src/main.cpp
                                       src/pointcloud_boxcrop_node.cpp)

ament_target_dependencies(pointcloud_boxcrop_node ${PACKAGE_DEPENDENCIES})

target_link_libraries(pointcloud_boxcrop_node ${PCL_LIBRARIES}
                      pointcloud_boxcrop_params)
target_include_directories(
  pointcloud_boxcrop_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)

install(TARGETS pointcloud_boxcrop_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
