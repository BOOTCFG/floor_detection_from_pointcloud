# Floor Detection From Point Cloud

This ROS2 package detects holes in ground point clouds. The package uses OpenCV to detect the holes. The package should be used with the `linefit_ground_segmentation_ros2` package.

## Description

This package provides a node, `floor_detection_from_pointcloud_node`, which takes in point cloud data (presumably from a LiDAR or similar sensor), and applies algorithms to detect the presence of holes in the ground.

The list of ROS parameters that the node uses are:

- **camera.range_near**: Range start for the camera (default: 0.5), 
- **camera.range_far**: Range end for the camera (default: 6.0),
- **camera.fov_degrees**: Field of view for the camera in degrees (default: 50.0),
- **camera.gravity_aligned_frame**: The frame aligned with gravity (default: camera_gravity_link),
- **resolution**: Resolution of the images (default: 0.1),
- **img_size**: Size of the images (default: 20.0),
- **show_debug_window**: Indicates if a debug window is shown or not (default: false),
- **use_sim_time**: If true, the system will use simulated time (default: true).

The package also includes a debug viewer which can show the results in a user-friendly manner.

## Topics

The node subscribes to the following topics for the input ground and obstacles pointcloud data:

- **/segmentation/ground**: This topic should publish messages of type `sensor_msgs/msg/PointCloud2`. The node uses this as the input for the ground pointcloud.

- **/segmentation/obstacle**: This topic should publish messages of type `sensor_msgs/msg/PointCloud2`. The node uses this as the input for the obstacles pointcloud.

Additionally, the node publishes the detected holes to the following topics:

- **detected_holes**: This topic publishes messages of type `sensor_msgs/msg/LaserScan`. It publishes the range of detected holes.

- **detected_holes_cloud**: This topic publishes messages of type `sensor_msgs/msg/PointCloud2`. It publishes the pointcloud of the detected holes.
