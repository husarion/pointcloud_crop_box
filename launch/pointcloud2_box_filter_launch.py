import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pointcloud2_box_filter'),
        'config',
        'pointcloud2_box_filter_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='pointcloud2_box_filter',
            executable='pointcloud2_box_filter_node',
            output='screen',
            parameters=[config],
        ),
    ])
