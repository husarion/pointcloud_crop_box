# pointcloud_crop_box

A ROS 2 package for filtering 3D point clouds using an axis-aligned cropping box.
It subscribes to a `sensor_msgs/msg/PointCloud2`, transforms it into a target frame, filters it using a configurable box, and publishes both the filtered cloud and a 3D bounding box message.

<p float="center">
  <img src=".docs/forward_ground.png" width="30%" />
  <img src=".docs/inner.png" width="30%" />
  <img src=".docs/forward_wall.png" width="30%" />
</p>


---

## Features

- Subscribes to raw point cloud data (e.g., from a LiDAR)
- Filters the cloud using a configurable 3D bounding box
- Supports TF2 transformation to a target frame
- Option to invert the box (i.e., remove inside instead of outside)
- Publishes the filtered cloud and a 3D bounding box marker

---

## ROS 2 Node

### `pointcloud_crop_box_node`

#### Subscribes

- **`<input_topic>`** (*sensor_msgs/msg/PointCloud2*)
  Input topic containing raw point cloud data (default: `/points_raw`)

#### Publishes

- **`<output_topic>`** (*sensor_msgs/msg/PointCloud2*)
  Filtered point cloud after box cropping (default: `/points_filtered`)

- **`~/filter_bounding_box`** (*vision_msgs/msg/BoundingBox3D*)
  The axis-aligned box used for filtering, for visualization or debugging

---

## Parameters

- `input_topic` [*string*, default: **"/points_raw"**]: Topic to subscribe for input PointCloud2 data.
- `output_topic` [*string*, default: **"/points_filtered"**]: Topic to publish filtered PointCloud2 data.
- `target_frame` [*string*, default: **"base_link"**]: Target TF frame to transform the point cloud into.
- `min_x` [*double*, default: **-1.0**]: Minimum X boundary of the crop box.
- `max_x` [*double*, default: **1.0**]: Maximum X boundary of the crop box.
- `min_y` [*double*, default: **-1.0**]: Minimum Y boundary of the crop box.
- `max_y` [*double*, default: **1.0**]: Maximum Y boundary of the crop box.
- `min_z` [*double*, default: **-1.0**]: Minimum Z boundary of the crop box.
- `max_z` [*double*, default: **1.0**]: Maximum Z boundary of the crop box.
- `negative` [*bool*, default: **false**]: If true, keeps points **outside** the crop box instead of inside.
- `visualize_bounding_box` [*bool*, default: **true**]: Whether to publish a visualization marker for the bounding box.

---

## Example Launch

To show the result of the filtering you can follow our autonomy example in [husarion-ugv-autonomy](github.com/husarion/husarion-ugv-autonomy) to get  measurements of pointcloud.

## Build from Source

### Create Workspace

```bash
mkdir ~/husarion_ws
cd ~/husarion_ws
git clone https://github.com/husarion/pointcloud_crop_box.git src/pointcloud_crop_box
```

### Build

```bash
sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to pointcloud_crop_box --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run configured launch:

```bash
source install/setup.bash
ros2 launch  pointcloud_crop_box pointcloud_crop_box_launch.py
```
