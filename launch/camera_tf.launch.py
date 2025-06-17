from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.2", "0", "0.13","0", "0",  "0", "lynx/mount_link", "camera_gravity_link"],
        # arguments=["0.2", "0", "0.13","3.141592" ,"1.570796", "1.570796",  "lynx/mount_link", "camera_gravity_link"],
        # arguments=["0.2", "0", "0.13", "-1.570796" ,"0.0", "-1.570796",  "lynx/mount_link", "camera_gravity_link"],
    )


    node2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0.31", "0", "camera_gravity_link", "camera_link"],
    )


    ld.add_action(node)
    ld.add_action(node2)

    return ld
