// Copyright 2016-2025 Aeva, Inc. All rights reserved.
// 
// Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
// code for terms and conditions that govern your use of this software. Any use, reproduction,
// disclosure, or distribution of this software and related documentation outside the terms of the
// EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

#include <iomanip>

#include "AevaPublisherNode.h"

namespace aeva {
namespace api {
namespace ros2_examples {

namespace {

// Queue size for the publishers.
constexpr int64_t kDefaultQueueSize{10};

// Used for topic names for Aeva RawData types.
// These message indices are used to open the ROS bag with Aeviz.
constexpr std::size_t kCalibrationMessageIndex{1};
constexpr std::size_t kTelemetryMessageIndex{3};
constexpr std::size_t kConfigMessageIndex{4};
constexpr std::size_t kLogMessageIndex{5};
constexpr std::size_t kObjectListMessageIndex{6};
constexpr std::size_t kPointCloudMessageIndex{7};
constexpr std::size_t kSPCDataMessageIndex{11};
constexpr std::size_t kHealthMessageIndex{12};
constexpr std::size_t kVelocityEstimateMessageIndex{13};
constexpr std::size_t kIMUMessageIndex{14};
constexpr std::size_t kVSE3DMessageIndex{16};
constexpr std::size_t kReconfigMessageIndex{17};
constexpr std::size_t kRawPacketMessageIndex{20};
constexpr std::size_t kNavigationSolutionMessageIndex{23};
constexpr std::size_t kSensorOdometryMessageIndex{35};
constexpr std::size_t kCPCPointGroupMessageIndex{45};

// Reserved sensor index for sensor fusion pipeline.
constexpr std::size_t kSensorFusionSensorIndex{0xFE};

// Topic names for ROS native types.
constexpr char kTopicPrefix[] = "/aeva/";
constexpr char kTransformTopic[] = "/tf";
// Topic names suffixes.
constexpr char kHealthSuffix[] = "/health";
constexpr char kIMUSuffix[] = "/imu";
constexpr char kDetectionArraySuffix[] = "/object_list/detections";
constexpr char kMarkerArraySuffix[] = "/object_list/markers";
constexpr char kOdometrySuffix[] = "/odometry";
constexpr char kPointCloudSuffix[] = "/point_cloud_compensated";
constexpr char kPointCloudMetadataSuffix[] = "/point_cloud_metadata";
constexpr char kTwistSuffix[] = "/twist";
constexpr char kMetadataTopic[] = "/metadata";

}  // namespace

AevaPublisherNode::AevaPublisherNode(const std::string& node_name, const std::string& sensor_id,
                                     const std::size_t sensor_index, const int sensor_index_offset,
                                     rclcpp::Logger& logger_in, bool publish_raw_topics,
                                     bool publish_ros_topics)
    : Node(node_name),
      sensor_id_(sensor_id),
      sensor_index_(sensor_index),
      sensor_index_offset_(sensor_index_offset),
      logger(logger_in),
      publish_raw_topics_(publish_raw_topics),
      publish_ros_topics_(publish_ros_topics) {
  if (api::kSensorFusionSensorId == sensor_id_ && kSensorFusionSensorIndex != sensor_index) {
    RCLCPP_FATAL_STREAM(logger, "Invalid sensor index for sensor fusion publisher, must be 0xFE");
    exit(1);
  }

  if (!publish_raw_topics_ && !publish_ros_topics_) {
    RCLCPP_FATAL_STREAM(logger, "Both raw topics and ROS topics cannot be disabled!");
    exit(1);
  }

  // Initialize ROS native type publishers.
  //
  // NOTE: There may be additional topics that get advertised for external sensors and sensor
  // fusion pipeline, however there will no data published on these topics. Eg: External GNSS may
  // advertise a point cloud topic but will never publish any point cloud data on it.
  if (publish_ros_topics_) {
    point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        GetTopicName(kPointCloudSuffix), kDefaultQueueSize);
    point_cloud_metadata_pub_ = create_publisher<aeva_msgs::msg::PointCloudMetadata>(
        GetTopicName(kPointCloudMetadataSuffix), kDefaultQueueSize);
    odometry_pub_ =
        create_publisher<nav_msgs::msg::Odometry>(GetTopicName(kOdometrySuffix), kDefaultQueueSize);
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        GetTopicName(kTwistSuffix), kDefaultQueueSize);
    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(kTransformTopic, kDefaultQueueSize);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(GetTopicName(kIMUSuffix), kDefaultQueueSize);
    health_pub_ =
        create_publisher<aeva_msgs::msg::Health>(GetTopicName(kHealthSuffix), kDefaultQueueSize);
    detection_pub_ = create_publisher<aeva_msgs::msg::Detection3DArray>(
        GetTopicName(kDetectionArraySuffix), kDefaultQueueSize);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        GetTopicName(kMarkerArraySuffix), kDefaultQueueSize);
  }

  if (publish_raw_topics_) {
    // Initialize ROS Aeva raw data publishers.
    raw_point_cloud_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kPointCloudMessageIndex, "PointCloud", "point_cloud_compensated"),
        kDefaultQueueSize);
    raw_calibration_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kCalibrationMessageIndex, "Calibration", "calibration"), kDefaultQueueSize);
    raw_vehicle_state_estimate_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kVSE3DMessageIndex, "VehicleStateEstimate", "S3d_vehicle_state_estimate"),
        kDefaultQueueSize);
    raw_sensor_fusion_vehicle_state_estimate_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kVSE3DMessageIndex, "VehicleStateEstimate",
                     "sensor_fusion_3d_vehicle_state_estimate"),
        kDefaultQueueSize);
    raw_velocity_estimate_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kVelocityEstimateMessageIndex, "VelocityEstimate", "velocity_estimate"),
        kDefaultQueueSize);
    raw_odometry_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kSensorOdometryMessageIndex, "Odometry", "sensor_odometry"),
        kDefaultQueueSize);
    raw_imu_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kIMUMessageIndex, "IMU", "imu"), kDefaultQueueSize);
    raw_health_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kHealthMessageIndex, "Health", "health"), kDefaultQueueSize);
    raw_navigation_solution_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kNavigationSolutionMessageIndex, "NavigationSolution", "navigation_solution"),
        kDefaultQueueSize);
    raw_object_list_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kObjectListMessageIndex, "ObjectList", "lidar_object_list"),
        kDefaultQueueSize);
    raw_sensor_fusion_object_list_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kObjectListMessageIndex, "ObjectList", "sensor_fusion_lidar_object_list"),
        kDefaultQueueSize);
    raw_spc_pointgroup_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kSPCDataMessageIndex, "RawData", "spc_pointgroup"), kDefaultQueueSize);
    raw_cpc_pointgroup_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kCPCPointGroupMessageIndex, "AtlasCpcData", "raw_cpc_pointgroup"),
        kDefaultQueueSize);
    raw_packet_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kRawPacketMessageIndex, "RawData", "raw_packet"), kDefaultQueueSize);
    raw_reconfig_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kReconfigMessageIndex, "Reconfig", "reconfig_client"), kDefaultQueueSize);
    raw_telemetry_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kTelemetryMessageIndex, "Telemetry", "telemetry"), kDefaultQueueSize);
    raw_config_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kConfigMessageIndex, "Config", "config"), kDefaultQueueSize);
    raw_log_pub_ = create_publisher<aeva_msgs::msg::RawData>(
        GetTopicName(kLogMessageIndex, "Log", "log"), kDefaultQueueSize);
    metadata_pub_ =
        create_publisher<std_msgs::msg::String>(GetMetadataTopicName(), kDefaultQueueSize);
  }
}

void AevaPublisherNode::PublishPointCloud(const sensor_msgs::msg::PointCloud2& message,
                                          const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    point_cloud_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishPointCloudMetadata(const aeva_msgs::msg::PointCloudMetadata& message,
                                                  const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    point_cloud_metadata_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishTransform(const tf2_msgs::msg::TFMessage& message,
                                         const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    tf_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishOdometry(const nav_msgs::msg::Odometry& message,
                                        const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    odometry_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishTwist(const geometry_msgs::msg::TwistWithCovarianceStamped& message,
                                     const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    twist_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishIMU(const sensor_msgs::msg::Imu& message,
                                   const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    imu_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishHealth(const aeva_msgs::msg::Health& message,
                                      const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    health_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishDetectionArray(const aeva_msgs::msg::Detection3DArray& message,
                                              const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    detection_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishMarkerArray(const visualization_msgs::msg::MarkerArray& message,
                                           const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    marker_pub_->publish(message);
  }
}

/* ROS Aeva raw data types publish callbacks */

void AevaPublisherNode::PublishRawPointCloud(const aeva_msgs::msg::RawData& message,
                                             const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_point_cloud_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawCalibration(const aeva_msgs::msg::RawData& message,
                                              const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_calibration_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawVehicleStateEstimate(const aeva_msgs::msg::RawData& message,
                                                       const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_vehicle_state_estimate_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawSensorFusionVehicleStateEstimate(
    const aeva_msgs::msg::RawData& message, const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_sensor_fusion_vehicle_state_estimate_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawVelocityEstimate(const aeva_msgs::msg::RawData& message,
                                                   const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_velocity_estimate_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawOdometry(const aeva_msgs::msg::RawData& message,
                                           const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_odometry_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawIMU(const aeva_msgs::msg::RawData& message,
                                      const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_imu_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawNavigationSolution(const aeva_msgs::msg::RawData& message,
                                                     const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_navigation_solution_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawObjectList(const aeva_msgs::msg::RawData& message,
                                             const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_object_list_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawSensorFusionObjectList(const aeva_msgs::msg::RawData& message,
                                                         const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_sensor_fusion_object_list_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawSPCPointGroup(const aeva_msgs::msg::RawData& message,
                                                const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_spc_pointgroup_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawCPCPointGroup(const aeva_msgs::msg::RawData& message,
                                                const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_cpc_pointgroup_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawReconfig(const aeva_msgs::msg::RawData& message,
                                           const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_reconfig_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawConfig(const aeva_msgs::msg::RawData& message,
                                         const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_config_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawLog(const aeva_msgs::msg::RawData& message,
                                      const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_log_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawHealth(const aeva_msgs::msg::RawData& message,
                                         const std::string sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_health_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawPacket(const aeva_msgs::msg::RawData& message,
                                         const std::string sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_packet_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishRawTelemetry(const aeva_msgs::msg::RawData& message,
                                            const std::string sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_telemetry_pub_->publish(message);
  }
}

void AevaPublisherNode::PublishMetadata(const std_msgs::msg::String& message) {
  if (publish_raw_topics_) {
    metadata_pub_->publish(message);
  }
}

std::string AevaPublisherNode::GetTopicName(const std::string& topic_suffix) const {
  std::stringstream topic_ss;
  topic_ss << kTopicPrefix << sensor_id_ << topic_suffix;
  return topic_ss.str();
}

std::string AevaPublisherNode::GetTopicName(std::size_t stream_index,
                                            const std::string& message_type,
                                            const std::string& stream_name) const {
  std::stringstream topic_ss;
  topic_ss << kTopicPrefix << "I" << std::hex << std::setw(6) << std::setfill('0')
           << (sensor_index_ << 16 | stream_index << 8) << "/" << message_type << "/"
           << stream_name;
  return topic_ss.str();
}

std::string AevaPublisherNode::GetMetadataTopicName() const {
  std::stringstream topic_ss;
  topic_ss << kTopicPrefix << "I" << std::hex << std::setw(6) << std::setfill('0')
           << (sensor_index_offset_ << 16) << kMetadataTopic;
  return topic_ss.str();
}

}  // namespace ros2_examples
}  // namespace api
}  // namespace aeva
