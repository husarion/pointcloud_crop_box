# pointcloud_boxcrop

A ROS 2 package for filtering 3D point clouds using an axis-aligned cropping box.
It subscribes to a `sensor_msgs/msg/PointCloud2`, transforms it into a target frame, filters it using a configurable box, and publishes both the filtered cloud and a 3D bounding box message.

<p float="left">
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

### `pointcloud_boxcrop_node`

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

All parameters are set via the `pointcloud_boxcrop_params` namespace.

| Name             | Type    | Default     | Description |
|------------------|---------|-------------|-------------|
| `input_topic`    | string  | `/points_raw`      | Topic to subscribe for input point cloud |
| `output_topic`   | string  | `/points_filtered` | Topic to publish filtered point cloud |
| `target_frame`   | string  | `base_link`        | Frame to which point cloud is transformed |
| `min_x`          | double  | `-1.0`       | Minimum X boundary of the crop box |
| `max_x`          | double  | `1.0`        | Maximum X boundary of the crop box |
| `min_y`          | double  | `-1.0`       | Minimum Y boundary of the crop box |
| `max_y`          | double  | `1.0`        | Maximum Y boundary of the crop box |
| `min_z`          | double  | `-1.0`       | Minimum Z boundary of the crop box |
| `max_z`          | double  | `1.0`        | Maximum Z boundary of the crop box |
| `negative`       | bool    | `false`      | If true, keep points **outside** the box instead of inside |

---

## Example Launch

```bash
ros2 run pointcloud_boxcrop pointcloud_boxcrop_node --ros-args \
  -p input_topic:=/velodyne_points \
  -p output_topic:=/points_filtered \
  -p target_frame:=base_link \
  -p min_x:=-2.0 -p max_x:=2.0 \
  -p min_y:=-1.0 -p max_y:=1.0 \
  -p min_z:=-1.5 -p max_z:=0.5 \
  -p negative:=false
