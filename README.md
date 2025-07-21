# pointcloud_boxcrop
A ROS 2 package that subscribes to a sensor_msgs/msg/PointCloud2 topic, filters the incoming point cloud using a 3D box crop filter, and publishes the filtered point cloud. Additionally, the node publishes a visualization of the bounding box used for cropping.
