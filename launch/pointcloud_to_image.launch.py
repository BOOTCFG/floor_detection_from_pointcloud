from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('pointcloud_to_image'),
        'config',
        'pointcloud_to_image.yaml'
    )

    node = Node(
        package='pointcloud_to_image',
        executable='pointcloud_to_image_node',
        name='pointcloud_to_image_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([node])