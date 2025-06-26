#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#define FLOOR 255
#define OBSTACLE 2
// #define UNKNOWN 0

class FloorDetectionFromPointCloudNode : public rclcpp::Node {
public:
  FloorDetectionFromPointCloudNode() : Node("floor_detection_from_pointcloud_node") {

    // Define parameters with defaults
    declare_parameter<float>("camera.range_near", 0.5f);
    declare_parameter<float>("camera.range_far", 8.0f);
    declare_parameter<float>("camera.fov_degrees", 60.0f);
    declare_parameter<float>("resolution", 0.1f);
    declare_parameter<float>("img_size", 20.0f);
    declare_parameter<bool>("show_debug_window", false);
    declare_parameter("camera.gravity_aligned_frame",
                      std::string("camera_gravity_link"));

    // Get params into class fields
    range_near_ = get_parameter("camera.range_near").as_double();
    range_far_ = get_parameter("camera.range_far").as_double();
    fov_degrees_ = get_parameter("camera.fov_degrees").as_double();
    resolution_ = get_parameter("resolution").as_double();
    img_size_ = get_parameter("img_size").as_double();
    show_debug_window_ = get_parameter("show_debug_window").as_bool();
    gravity_aligned_frame_ =
        get_parameter("camera.gravity_aligned_frame").as_string();

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/segmentation/ground", rclcpp::SensorDataQoS(),
        std::bind(&FloorDetectionFromPointCloudNode::topic_callback, this,
                  std::placeholders::_1));

    subscription_obstacles_ =
        create_subscription<sensor_msgs::msg::PointCloud2>(
            "/segmentation/obstacle", rclcpp::SensorDataQoS(),
            std::bind(&FloorDetectionFromPointCloudNode::topic_obstacles_callback, this,
                      std::placeholders::_1));

    laser_pub_ =
        create_publisher<sensor_msgs::msg::LaserScan>("detected_holes", 10);
    holes_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "detected_holes_cloud", rclcpp::SensorDataQoS());

    // TF listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cv::namedWindow("PointCloud Image", cv::WINDOW_AUTOSIZE);
  }

  ~FloorDetectionFromPointCloudNode() { cv::destroyAllWindows(); }

private:
  float range_near_, range_far_, fov_degrees_, resolution_, img_size_;
  bool show_debug_window_;
  std::string gravity_aligned_frame_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      subscription_obstacles_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr holes_cloud_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  sensor_msgs::msg::PointCloud2::SharedPtr msg_ground_;
  sensor_msgs::msg::PointCloud2::SharedPtr msg_obstacles_;

  std::mutex processing_mutex_;

  void drawCameraFOV(cv::Mat &image) {
    float half_fov_rad = (fov_degrees_ / 2.0f) * (M_PI / 180.0f);
    std::vector<cv::Point> pts_pixel;

    std::vector<cv::Point2f> pts_camera{
        cv::Point2f(range_near_, -range_near_ * tan(half_fov_rad)),
        cv::Point2f(range_near_, range_near_ * tan(half_fov_rad)),
        cv::Point2f(range_far_, range_far_ * tan(half_fov_rad)),
        cv::Point2f(range_far_, -range_far_ * tan(half_fov_rad))};

    for (auto &pt : pts_camera) {
      int x = int((pt.x + img_size_ / 2) / resolution_);
      int y = image.rows - int((pt.y + img_size_ / 2) / resolution_) - 1;
      pts_pixel.emplace_back(cv::Point(x, y));
    }

    cv::polylines(image, pts_pixel, true, cv::Scalar(0, 255, 255), 1);
  }

  void raytraceFOVandDetectHoles(cv::Mat &image) {
    cv::Point origin(int(img_size_ / (2 * resolution_)),
                     int(img_size_ / (2 * resolution_)));
    float half_fov_rad = (fov_degrees_ / 2.0f) * M_PI / 180.0f;
    float angle_increment = M_PI / 180.0f / 2; // 0.5 degree increments

    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = gravity_aligned_frame_;
    scan.header.stamp = now();
    scan.angle_min = -half_fov_rad;
    scan.angle_max = half_fov_rad;
    scan.angle_increment = angle_increment;
    scan.range_min = range_near_;
    scan.range_max = range_far_;

    for (float angle = -half_fov_rad; angle <= half_fov_rad;
         angle += angle_increment) {
      float r = range_far_;

      cv::Point endpoint(origin.x + r * cos(angle) / resolution_,
                         origin.y - r * sin(angle) / resolution_);

      cv::LineIterator it(image, origin, endpoint, 8);

      bool found_floor = false;
      float detected_range = scan.range_max;

      for (int i = 0; i < it.count; ++i, ++it) {
        cv::Point p = it.pos();
        if (p.x < 0 || p.x >= image.cols || p.y < 0 || p.y >= image.rows)
          break;

        cv::Vec3b color = image.at<cv::Vec3b>(p);
        if (color == cv::Vec3b(FLOOR, 0, 0))
          break; // obstacle
        if (color == cv::Vec3b(0, FLOOR, 0))
          found_floor = true;
        else if (found_floor && color == cv::Vec3b(0, 0, 0)) {
          detected_range =
              hypot((p.x - origin.x), (p.y - origin.y)) * resolution_;
          cv::circle(image, p, 2, cv::Scalar(0, 0, 255), -1);
          break;
        }
      }
      scan.ranges.push_back(detected_range);
    }
    laser_pub_->publish(scan);
  }

  void raytraceFOVandPublishHolesPointCloud(cv::Mat &image) {
    cv::Point origin(int(img_size_ / (2 * resolution_)),
                     int(img_size_ / (2 * resolution_)));
    float half_fov_rad = (fov_degrees_ / 2.0f) * (M_PI / 180.0f);
    float angle_increment = (1.0f) * M_PI / 180.0f;

    pcl::PointCloud<pcl::PointXYZ> hole_cloud;
    hole_cloud.header.frame_id = "camera_gravity_link";
    hole_cloud.points.clear();

    for (float angle = -half_fov_rad; angle <= half_fov_rad;
         angle += angle_increment) {
      float r = range_far_;
      cv::Point endpoint(origin.x + r * cos(angle) / resolution_,
                         origin.y - r * sin(angle) / resolution_);

      cv::LineIterator it(image, origin, endpoint, 8);
      bool found_floor = false;

      for (int i = 0; i < it.count; ++i, ++it) {
        cv::Point p = it.pos();
        if (p.x < 0 || p.x >= image.cols || p.y < 0 || p.y >= image.rows)
          continue;

        cv::Vec3b color = image.at<cv::Vec3b>(p);
        if (color == cv::Vec3b(FLOOR, 0, 0)) // obstacle
          break;

        if (color == cv::Vec3b(0, FLOOR, 0))
          found_floor = true;
        else if (found_floor && color == cv::Vec3b(0, 0, 0)) {
          float hole_x = (p.x - origin.x) * resolution_;
          float hole_y = -(p.y - origin.y) * resolution_;

          // hole_cloud.points.emplace_back(hole_x, hole_y, 0.0f);
          hole_cloud.points.emplace_back(hole_x, hole_y, 0.0f); //TODO: remove hardcoded z value

          cv::circle(image, p, 2, cv::Scalar(0, 0, 255), -1);
          // break;
        }
      }
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(hole_cloud, cloud_msg);
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "camera_gravity_link";
    holes_cloud_pub_->publish(cloud_msg);
  }

  void process_costmap() {

    std::unique_lock<std::mutex> lock(processing_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      RCLCPP_WARN(this->get_logger(), "process_costmap() already running, skipping this callback");
      return;
    }

    // Convert ROS2 PointCloud2 message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
    pcl::fromROSMsg(*msg_ground_, cloud);

    if (cloud.empty() || cloud.width == 0 || cloud.points.empty()) {
      RCLCPP_DEBUG(get_logger(), "Ground cloud empty or invalid; skipping processing");
      return; 
    }

    geometry_msgs::msg::TransformStamped tf_stamped;

    Eigen::Affine3d tf;
    try {
      tf_stamped = tf_buffer_->lookupTransform(gravity_aligned_frame_,
                                               msg_ground_->header.frame_id,
                                               msg_ground_->header.stamp);
      // Remove translation part.
      tf_stamped.transform.translation.x = 0;
      tf_stamped.transform.translation.y = 0;
      tf_stamped.transform.translation.z = 0;

      Eigen::Quaterniond rotation(
          tf_stamped.transform.rotation.w, tf_stamped.transform.rotation.x,
          tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z);

      tf = Eigen::Affine3d(rotation);

      // tf::transformMsgToEigen(tf_stamped.transform, tf);
      pcl::transformPointCloud(cloud, cloud_transformed, tf);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to transform point cloud into "
                  "gravity frame: %s",
                  ex.what());
      return;
    }

    cloud = cloud_transformed;
    pcl::PointCloud<pcl::PointXYZ> cloud_obstacles;
    pcl::PointCloud<pcl::PointXYZ> cloud_obstacles_transformed;
    if (!msg_obstacles_) {
      RCLCPP_WARN(this->get_logger(),
                  "Received empty point cloud message for obstacles");
      return;
    }
    pcl::fromROSMsg(*msg_obstacles_, cloud_obstacles);
    pcl::transformPointCloud(cloud_obstacles, cloud_obstacles_transformed, tf);
    cloud_obstacles = cloud_obstacles_transformed;

    if (cloud.empty() || cloud_obstacles.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty cloud");
      return;
    }

    int img_dim = int(img_size_ / resolution_);
    cv::Mat img = cv::Mat::zeros(img_dim, img_dim, CV_8UC3);

    // floor pointcloud
    for (const auto &point : cloud.points) {
      int x_pixel = int((point.x + img_size_ / 2) / resolution_);
      int y_pixel = int((point.y + img_size_ / 2) / resolution_);

      // Skip points which fall out of the defined image boundaries
      if (x_pixel >= 0 && x_pixel < img.cols && y_pixel >= 0 &&
          y_pixel < img.rows) {

        img.at<cv::Vec3b>(img.rows - y_pixel - 1, x_pixel) =
            cv::Vec3b(0, FLOOR, 0); // Floor color
      }
    }


    cv::Mat closing;
    cv::morphologyEx(img, closing, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    // obstacles pointcloud
    for (const auto &point : cloud_obstacles.points) {
      int x_pixel = int((point.x + img_size_ / 2) / resolution_);
      int y_pixel = int((point.y + img_size_ / 2) / resolution_);

      int row = closing.rows - y_pixel - 1;
      int col = x_pixel;

      if (col >= 0 && col < closing.cols && row >= 0 && row < closing.rows) {
          closing.at<cv::Vec3b>(row, col) = cv::Vec3b(FLOOR, 0, 0);
      }
    }


    // This part isnt working due to color merging
    std::vector<cv::Mat> channels(3);
    cv::split(closing, channels);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    // cv::morphologyEx(channels[0], channels[0], cv::MORPH_OPEN, kernel);
    // cv::dilate(channels[0], channels[0], kernel);
    cv::Mat result_img;
    cv::merge(channels, result_img);  


    // raytraceFOVandDetectHoles(result_img);
    raytraceFOVandPublishHolesPointCloud(result_img);

    if (show_debug_window_) {
      drawCameraFOV(result_img);

      double scale = 4.0;
      cv::Mat scaled;
      cv::resize(result_img, scaled, cv::Size(), scale, scale,
                 cv::INTER_NEAREST);

      cv::imshow("PointCloud Image", scaled);
      cv::waitKey(1);
    }
  }

  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    msg_ground_ = msg;
    process_costmap();
  }

  void
  topic_obstacles_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    msg_obstacles_ = msg;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FloorDetectionFromPointCloudNode>());
  rclcpp::shutdown();
  return 0;
}