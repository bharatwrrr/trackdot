// Copyright 2016-2025 Aeva, Inc. All rights reserved.
// 
// Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
// code for terms and conditions that govern your use of this software. Any use, reproduction,
// disclosure, or distribution of this software and related documentation outside the terms of the
// EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

#include "AevaAPICallbacks.h"

namespace aeva {
namespace api {
namespace ros2_examples {

void PointCloudCallback(const api::ROS2PointCloudWrapper& wrapper,
                        std::vector<std::shared_ptr<AevaPublisherNode>>& publishers) {
  for (auto& publisher : publishers) {
    publisher->PublishPointCloud(wrapper.point_cloud_, wrapper.header_.sensor_id_);
    publisher->PublishPointCloudMetadata(wrapper.metadata_, wrapper.header_.sensor_id_);
    publisher->PublishRawPointCloud(wrapper.raw_data_, wrapper.header_.sensor_id_);
  }
}

void CalibrationCallback(const api::ROS2CalibrationWrapper& wrapper,
                         std::vector<std::shared_ptr<AevaPublisherNode>>& publishers) {
  for (auto& publisher : publishers) {
    publisher->PublishTransform(wrapper.transform_, wrapper.header_.sensor_id_);
    publisher->PublishRawCalibration(wrapper.raw_data_, wrapper.header_.sensor_id_);
  }
}

void VehicleStateEstimateCallback(const api::ROS2VehicleStateEstimateWrapper& wrapper,
                                  std::vector<std::shared_ptr<AevaPublisherNode>>& publishers,
                                  const std::string& main_lidar_sensor_id,
                                  bool sensor_fusion_enabled) {
  for (auto& publisher : publishers) {
    if (!sensor_fusion_enabled && wrapper.header_.sensor_id_ == main_lidar_sensor_id) {
      // This is the callback for individual sensor state estimates without fusion.
      // NOTE: There could be multiple sensors estimating the same world to vehicle transform or
      // vehicle odometry and this may cause the point cloud data to appear as flickering or
      // drifting apart from each other on Rviz. We recommend filtering out the duplicate world to
      // vehicle transform using the sensor_id when using multiple sensors and keep the transform
      // estimates only for one of the sensors (preferably the one running ICP motion estimator,
      // which is the first sensor in this example). In general, ICP motion estimation works best on
      // a wide FOV sensor facing forwards. Alternatively you may enable sensor fusion when using
      // multiple sensors which will estimate a single fused world to vehicle odometry/transform.
      publisher->PublishTransform(wrapper.transform_, wrapper.header_.sensor_id_);
      publisher->PublishOdometry(wrapper.odometry_, wrapper.header_.sensor_id_);
    }
    publisher->PublishRawVehicleStateEstimate(wrapper.raw_data_, wrapper.header_.sensor_id_);
  }
}

void SensorFusionVehicleStateEstimateCallback(
    const api::ROS2VehicleStateEstimateWrapper& wrapper,
    std::vector<std::shared_ptr<AevaPublisherNode>>& publishers, bool sensor_fusion_enabled) {
  for (auto& publisher : publishers) {
    if (sensor_fusion_enabled && publisher->GetSensorID() == api::kSensorFusionSensorId) {
      // This is the callback for state estimates with sensor fusion enabled.
      publisher->PublishTransform(wrapper.transform_, wrapper.header_.sensor_id_);
      publisher->PublishOdometry(wrapper.odometry_, wrapper.header_.sensor_id_);
      publisher->PublishRawSensorFusionVehicleStateEstimate(wrapper.raw_data_,
                                                            wrapper.header_.sensor_id_);
    }
  }
}

void VelocityEstimateCallback(const api::ROS2VelocityEstimateWrapper& wrapper,
                              std::vector<std::shared_ptr<AevaPublisherNode>>& publishers) {
  for (auto& publisher : publishers) {
    publisher->PublishTwist(wrapper.twist_, wrapper.header_.sensor_id_);
    publisher->PublishRawVelocityEstimate(wrapper.raw_data_, wrapper.header_.sensor_id_);
  }
}

void OdometryCallback(const api::ROS2OdometryWrapper& wrapper,
                      std::vector<std::shared_ptr<AevaPublisherNode>>& publishers) {
  for (auto& publisher : publishers) {
    // The odometry here (from Atlas) will always be in sensor frame.
    publisher->PublishOdometry(wrapper.odometry_, wrapper.header_.sensor_id_);
    publisher->PublishRawOdometry(wrapper.raw_data_, wrapper.header_.sensor_id_);
    publisher->PublishTransform(wrapper.transform_, wrapper.header_.sensor_id_);
  }
}

void IMUCallback(const api::ROS2IMUWrapper& wrapper,
                 std::vector<std::shared_ptr<AevaPublisherNode>>& publishers) {
  for (auto& publisher : publishers) {
    publisher->PublishIMU(wrapper.imu_, wrapper.header_.sensor_id_);
    publisher->PublishRawIMU(wrapper.raw_data_, wrapper.header_.sensor_id_);
  }
}

void HealthCallback(const api::ROS2HealthWrapper& wrapper,
                    std::vector<std::shared_ptr<AevaPublisherNode>>& publishers) {
  for (auto& publisher : publishers) {
    publisher->PublishHealth(wrapper.health_, wrapper.header_.sensor_id_);
    publisher->PublishRawHealth(wrapper.raw_data_, wrapper.header_.sensor_id_);
  }
}

void NavigationSolutionCallback(const api::ROS2NavigationSolutionWrapper& wrapper,
                                std::vector<std::shared_ptr<AevaPublisherNode>>& publishers) {
  for (auto& publisher : publishers) {
    publisher->PublishRawNavigationSolution(wrapper.raw_data_, wrapper.header_.sensor_id_);
  }
}

void ObjectListCallback(const api::ROS2ObjectListWrapper& wrapper,
                        std::vector<std::shared_ptr<AevaPublisherNode>>& publishers,
                        bool sensor_fusion_enabled) {
  for (auto& publisher : publishers) {
    if (!sensor_fusion_enabled) {
      // This is the callback for individual sensor detections without fusion.
      publisher->PublishDetectionArray(wrapper.detection_array_, wrapper.header_.sensor_id_);
      // Serializing visualization marker messages may cause a crash in ROS2 Humble.
      // publisher->PublishMarkerArray(wrapper.marker_array_, wrapper.header_.sensor_id_);
    }
    publisher->PublishRawObjectList(wrapper.raw_data_, wrapper.header_.sensor_id_);
  }
}

void SensorFusionObjectListCallback(const api::ROS2ObjectListWrapper& wrapper,
                                    std::vector<std::shared_ptr<AevaPublisherNode>>& publishers,
                                    bool sensor_fusion_enabled) {
  for (auto& publisher : publishers) {
    if (sensor_fusion_enabled && publisher->GetSensorID() == api::kSensorFusionSensorId) {
      // This is the callback for detection with sensor fusion enabled.
      publisher->PublishDetectionArray(wrapper.detection_array_, wrapper.header_.sensor_id_);
      // Serializing visualization marker messages may cause a crash in ROS2 Humble.
      // publisher->PublishMarkerArray(wrapper.marker_array_, wrapper.header_.sensor_id_);
      publisher->PublishRawSensorFusionObjectList(wrapper.raw_data_, wrapper.header_.sensor_id_);
    }
  }
}

void SPCPointGroupCallback(const api::ROS2RawDataWrapper& wrapper,
                           std::vector<std::shared_ptr<AevaPublisherNode>>& publishers) {
  for (auto& publisher : publishers) {
    publisher->PublishRawSPCPointGroup(wrapper.raw_data_, wrapper.header_.sensor_id_);
  }
}

void CPCPointGroupCallback(const api::ROS2RawDataWrapper& wrapper,
                           std::vector<std::shared_ptr<AevaPublisherNode>>& publishers) {
  for (auto& publisher : publishers) {
    publisher->PublishRawCPCPointGroup(wrapper.raw_data_, wrapper.header_.sensor_id_);
  }
}

void RawPacketCallback(const api::ROS2RawDataWrapper& wrapper,
                       std::vector<std::shared_ptr<AevaPublisherNode>>& publishers) {
  for (auto& publisher : publishers) {
    publisher->PublishRawPacket(wrapper.raw_data_, wrapper.header_.sensor_id_);
  }
}

void ReconfigCallback(const api::ROS2RawDataWrapper& wrapper,
                      std::vector<std::shared_ptr<AevaPublisherNode>>& publishers) {
  for (auto& publisher : publishers) {
    publisher->PublishRawReconfig(wrapper.raw_data_, wrapper.header_.sensor_id_);
  }
}

void TelemetryCallback(const api::ROS2TelemetryWrapper& wrapper,
                       std::vector<std::shared_ptr<AevaPublisherNode>>& publishers) {
  for (auto& publisher : publishers) {
    publisher->PublishRawTelemetry(wrapper.raw_data_, wrapper.header_.sensor_id_);
  }
}

void ConfigCallback(const api::ROS2RawDataWrapper& wrapper,
                    std::vector<std::shared_ptr<AevaPublisherNode>>& publishers) {
  for (auto& publisher : publishers) {
    publisher->PublishRawConfig(wrapper.raw_data_, wrapper.header_.sensor_id_);
  }
}

void LogCallback(const api::ROS2RawDataWrapper& wrapper,
                 std::vector<std::shared_ptr<AevaPublisherNode>>& publishers) {
  for (auto& publisher : publishers) {
    publisher->PublishRawLog(wrapper.raw_data_, wrapper.header_.sensor_id_);
  }
}

void MetadataCallback(const std_msgs::msg::String& metadata,
                      std::vector<std::shared_ptr<AevaPublisherNode>>& publishers) {
  for (auto& publisher : publishers) {
    publisher->PublishMetadata(metadata);
  }
}

}  // namespace ros2_examples
}  // namespace api
}  // namespace aeva
