// lidar_camera_overlay_node.cpp

#include <memory>
#include <vector>
#include <cmath>
#include <algorithm> // For std::clamp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"


#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <limits> // Include for std::numeric_limits


using std::placeholders::_1;
using std::placeholders::_2;

class LidarCameraOverlayNode : public rclcpp::Node
{
public:
  LidarCameraOverlayNode()
  : Node("lidar_camera_overlay_node")
  {
    // --- 1. Parameter Declarations ---
    // Intrinsics
    camera_fx_ = declare_parameter<double>("camera.fx", 1200.0);
    camera_fy_ = declare_parameter<double>("camera.fy", 1200.0);
    camera_cx_ = declare_parameter<double>("camera.cx", 960.0);
    camera_cy_ = declare_parameter<double>("camera.cy", 600.0);
    image_width_  = declare_parameter<int>("camera.width", 1920);
    image_height_ = declare_parameter<int>("camera.height", 1200);

    // Extrinsics: lidar -> camera
    std::vector<double> t_vec = declare_parameter<std::vector<double>>(
      "extrinsics.translation", {-0.2, 0.0, 0.2}); 
    std::vector<double> rpy_vec = declare_parameter<std::vector<double>>(
      "extrinsics.rotation_rpy", {0.0, 0.0, 0.0}); 

    // Topics & Sync
    camera_topic_ = declare_parameter<std::string>("topics.camera_image", "/triton_left/images");
    lidar_topic_  = declare_parameter<std::string>("topics.lidar_points", "/lidar_points");
    overlay_topic_ = declare_parameter<std::string>("topics.overlay_image", "/fusion/overlay_image");
    max_time_diff_ = declare_parameter<double>("sync.max_time_diff_sec", 0.05); // 50 ms

    // --- 2. Build Extrinsic (R, t) ---
    // Build rotation matrix from roll, pitch, yaw (RPY)
    double roll  = rpy_vec[0];
    double pitch = rpy_vec[1];
    double yaw   = rpy_vec[2];

    Eigen::Matrix3f R_roll, R_pitch, R_yaw;
    R_roll <<
      1,            0,             0,
      0, std::cos(roll), -std::sin(roll),
      0, std::sin(roll),  std::cos(roll);

    R_pitch <<
      std::cos(pitch), 0, std::sin(pitch),
      0,               1,             0,
      -std::sin(pitch), 0, std::cos(pitch);

    //TODO: this is not standard yaw rotation. i had to change the sign because of mismatch. change it back once mismatch is gone.
    R_yaw <<
      -std::cos(yaw), -std::sin(yaw), 0,
      std::sin(yaw),  -std::cos(yaw), 0,
      0,              0,             1;

    // R = R_yaw * R_pitch * R_roll (Z-Y-X extrinsic rotation order)
    R_ = R_yaw * R_pitch * R_roll;

    t_ = Eigen::Vector3f(
      static_cast<float>(t_vec[0]),
      static_cast<float>(t_vec[1]),
      static_cast<float>(t_vec[2])
    );

    RCLCPP_INFO(get_logger(), "Lidar-Camera overlay node initialized.");

    // --- 3. Create Publishers and Subscribers ---
    overlay_pub_ = create_publisher<sensor_msgs::msg::Image>(overlay_topic_, 10);

    // // Using rclcpp::SensorDataQoS() for image and point cloud topics is common
    // image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    //   camera_topic_, rclcpp::SensorDataQoS(),
    //   std::bind(&LidarCameraOverlayNode::imageCallback, this, _1));
 
    // lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    //   lidar_topic_, rclcpp::SensorDataQoS(),
    //   std::bind(&LidarCameraOverlayNode::lidarCallback, this, _1));

    rclcpp::QoS sensor_qos = rclcpp::QoS(10).best_effort();
    image_sub_.subscribe(this, camera_topic_, sensor_qos.get_rmw_qos_profile());
    lidar_sub_.subscribe(this, lidar_topic_, sensor_qos.get_rmw_qos_profile());
    uint32_t queue_size = 10;
    sync_ = std::make_shared<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::Image,
          sensor_msgs::msg::PointCloud2>>>(
            message_filters::sync_policies::ApproximateTime<
              sensor_msgs::msg::Image,
              sensor_msgs::msg::PointCloud2>(queue_size),
            image_sub_,
            lidar_sub_);
    sync_->setAgePenalty(10.0); // Optional: penalize older messages
    sync_->registerCallback(
        std::bind(&LidarCameraOverlayNode::doOverlay, this, _1, _2));

    
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    last_image_ = msg;
    tryOverlay();
  }

  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    last_cloud_ = msg;
    tryOverlay();
  }

  cv::Scalar rvizColormap(float n)
  {
      n = std::clamp(n, 0.0f, 1.0f);

      float r, g, b;

      if (n < 0.25f) {
          // red → yellow
          r = 255;
          g = 4.0f * n * 255.0f;
          b = 0;
      }
      else if (n < 0.5f) {
          // yellow → green
          float t = (n - 0.25f) * 4.0f;
          r = (1.0f - t) * 255.0f;
          g = 255;
          b = 0;
      }
      else if (n < 0.75f) {
          // green → cyan
          float t = (n - 0.5f) * 4.0f;
          r = 0;
          g = 255;
          b = t * 255.0f;
      }
      else {
          // cyan → blue
          float t = (n - 0.75f) * 4.0f;
          r = 0;
          g = (1.0f - t) * 255.0f;
          b = 255;
      }

      return cv::Scalar(b, g, r); // OpenCV uses BGR
  }

  void tryOverlay()
  {
    if (!last_image_ || !last_cloud_) {
      return;
    }

    // Check time difference for simple synchronization
    rclcpp::Time t_img(last_image_->header.stamp);
    rclcpp::Time t_cloud(last_cloud_->header.stamp);
    double dt = std::fabs((t_img - t_cloud).seconds());
    
    if (dt > max_time_diff_) {
      // Too far apart in time; skip overlay
      return;
    }

    // Execute the overlay with synchronized data
    doOverlay(last_image_, last_cloud_);
  }

  void doOverlay(
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 10000, "Sync callbacks with %u and %u as timestamps.",
                    image_msg->header.stamp,
                    cloud_msg->header.stamp);
        // Convert ROS Image -> OpenCV Mat (BGR8)
        float max_intensity = 0.0f;
        float min_intensity = 2.0f;
        // uint8_t vlow_intensity = 0;
        // uint8_t vhigh_intensity = 0;
        // uint8_t low_intensity = 0;
        // uint8_t high_intensity = 0;
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // Note: Your image topic publishes rgb8, converting to bgr8 for OpenCV
            cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8"); 
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat& img = cv_ptr->image;        
        const float MIN_INTENSITY = 0.0f; // Min intensity for scaling
        const float MAX_INTENSITY = 2.0f; // Max intensity for scaling

        // --- 4. Projection Loop ---
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*cloud_msg, "intensity");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
            // Lidar Point in Lidar Frame (X: Left, Y: Up, Z: Forward)
            float x_l = *iter_x; 
            float y_l = *iter_y; 
            float z_l = *iter_z; 
            float intensity = *iter_intensity;
            if (intensity > max_intensity) {
                max_intensity = intensity;
            }
            if (intensity < min_intensity) {
                min_intensity = intensity;
            }

            if (!std::isfinite(x_l) || !std::isfinite(y_l) || !std::isfinite(z_l))
                continue;

            // 4a. Transform Lidar Frame Point to Camera Frame Point
            Eigen::Vector3f p_lidar(x_l, y_l, z_l);
            Eigen::Vector3f p_cam = R_ * p_lidar + t_;

            // Camera Frame Axes (Standard Convention: X_C: Right, Y_C: Down, Z_C: Forward/Depth)
            float X_C = p_cam[0]; 
            float Y_C = p_cam[1]; 
            float Z_C = p_cam[2]; // Depth!

            // 4b. Check Depth and Visibility
            // Must be in front of camera (Z_C > 0)
            if (Z_C <= 0.1f)
                continue;

            // 4c. Pinhole Projection (No Distortion)
            // Normalized coordinates (x_n = X_C / Z_C, y_n = Y_C / Z_C)
            // Pixel coordinates: u = fx * x_n + cx, v = fy * y_n + cy
            float u = (static_cast<float>(camera_fx_) * (X_C / Z_C)) + static_cast<float>(camera_cx_);
            float v = (static_cast<float>(camera_fy_) * (Y_C / Z_C)) + static_cast<float>(camera_cy_);

      
            float n = (intensity - MIN_INTENSITY) / (MAX_INTENSITY - MIN_INTENSITY);
            cv::Scalar point_color = rvizColormap(n);
            

            // 2. Draw Point
            if (u >= 0 && u < img.cols && v >= 0 && v < img.rows) {
                // if (point_color[2]<65.0f){
                //   vlow_intensity++;
                // }
                // else if (point_color[2]>195.0f){
                //   vhigh_intensity++;
                // }
                // else if (point_color[2]>=65.0f && point_color[2]<=195.0f){
                //   low_intensity++;
                // }
                // else {
                //   high_intensity++;
                // }

                // if (intensity < 0.5f){
                //   vlow_intensity++;
                // }
                // else if (intensity > 1.5f) {
                //   vhigh_intensity++;
                // }
                // else if (intensity >= 0.5f && intensity <= 1.5f){
                //   low_intensity++;
                // }
                // else {
                //   high_intensity++;
                // }
               
                cv::circle(img, cv::Point(u, v), 2, point_color, -1);
            }
        }
        // RCLCPP_INFO(get_logger(), "Intensity Stats - Max: %.2f, Min: %.2f", max_intensity, min_intensity);
        // RCLCPP_INFO(get_logger(), "Red Intensity Counts - <65: %d, >195: %d, 65~195: %d, Other: %d", 
        //              vlow_intensity, vhigh_intensity, low_intensity, high_intensity);
        // RCLCPP_INFO(get_logger(), "Intensity Counts - <0.5: %d, >1.5: %d, 0.5~1.5: %d, Other: %d", 
        //              vlow_intensity, vhigh_intensity, low_intensity, high_intensity);

        // --- 5. Publish Result ---
        auto out_msg = cv_ptr->toImageMsg();
        out_msg->header = image_msg->header;  // preserve timestamp & frame_id
        overlay_pub_->publish(*out_msg);
  }

  // Parameters
  double camera_fx_, camera_fy_, camera_cx_, camera_cy_;
  int image_width_, image_height_;
  double max_time_diff_;
  std::string camera_topic_, lidar_topic_, overlay_topic_;

  // Intrinsics (only parameters are used for pinhole projection) and Extrinsics (R, t)
  Eigen::Matrix3f R_;
  Eigen::Vector3f t_;

  // // Subscriptions and publisher
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;
  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::PointCloud2>>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;

  // Last received messages for manual sync
  sensor_msgs::msg::Image::SharedPtr last_image_;
  sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarCameraOverlayNode>();
  // rclcpp::spin(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin(); // This will use multiple threads!
  rclcpp::shutdown();
  return 0;
}