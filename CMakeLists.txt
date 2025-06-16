cmake_minimum_required(VERSION 3.10)
project(floor_detection_from_pointcloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto)
# find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(pcl_conversions REQUIRED)
# find_package(PCL REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)



ament_auto_find_build_dependencies()
ament_auto_add_executable(floor_detection_from_pointcloud_node src/floor_detection_from_pointcloud_node.cpp)
ament_target_dependencies(floor_detection_from_pointcloud_node
  # rclcpp
  # sensor_msgs
  # pcl_conversions
  cv_bridge
  OpenCV
  # PCL
)


ament_auto_package(INSTALL_TO_SHARE launch config)

# install(TARGETS
#   floor_detection_from_pointcloud_node
#   DESTINATION lib/${PROJECT_NAME}/
# )

# install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()