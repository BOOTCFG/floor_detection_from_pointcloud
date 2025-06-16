from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('floor_detection_from_pointcloud'),
        'config',
        'floor_detection_from_pointcloud.yaml'
    )

    node = Node(
        package='floor_detection_from_pointcloud',
        executable='floor_detection_from_pointcloud_node',
        name='floor_detection_from_pointcloud_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([node])