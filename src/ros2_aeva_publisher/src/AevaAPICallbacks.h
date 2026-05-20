// Copyright 2016-2025 Aeva, Inc. All rights reserved.
// 
// Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
// code for terms and conditions that govern your use of this software. Any use, reproduction,
// disclosure, or distribution of this software and related documentation outside the terms of the
// EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "AevaPublisherNode.h"

namespace aeva {
namespace api {
namespace ros2_examples {

/// \brief Callback function that receives the PointCloud wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void PointCloudCallback(const api::ROS2PointCloudWrapper& wrapper,
                        std::vector<std::shared_ptr<AevaPublisherNode>>& publishers);

/// \brief Callback function that receives the Calibration wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void CalibrationCallback(const api::ROS2CalibrationWrapper& wrapper,
                         std::vector<std::shared_ptr<AevaPublisherNode>>& publishers);

/// \brief Callback function that receives the VehicleStateEstimate wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \note Only supported for Aeries II. See OdometryCallback for Atlas.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
/// \param sensor_fusion_enabled Indicates if sensor fusion is enabled in the Aeva API.
void VehicleStateEstimateCallback(const api::ROS2VehicleStateEstimateWrapper& wrapper,
                                  std::vector<std::shared_ptr<AevaPublisherNode>>& publishers,
                                  const std::string& main_lidar_sensor_id,
                                  bool sensor_fusion_enabled = false);

/// \brief Callback function that receives the SensorFusionVehicleStateEstimate wrapper
///        from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \note Only supported for Aeries II.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
/// \param sensor_fusion_enabled Indicates if sensor fusion is enabled in the Aeva API.
void SensorFusionVehicleStateEstimateCallback(
    const api::ROS2VehicleStateEstimateWrapper& wrapper,
    std::vector<std::shared_ptr<AevaPublisherNode>>& publishers,
    bool sensor_fusion_enabled = false);

/// \brief Callback function that receives the VelocityEstimate wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \note Only supported for Aeries II. See OdometryCallback for Atlas.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void VelocityEstimateCallback(const api::ROS2VelocityEstimateWrapper& wrapper,
                              std::vector<std::shared_ptr<AevaPublisherNode>>& publishers);

/// \brief Callback function that receives the Odometry wrapper from the Aeva API.
/// \note Only supported for Atlas. See VehicleStateEstimateCallback for Aeries II.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void OdometryCallback(const api::ROS2OdometryWrapper& wrapper,
                      std::vector<std::shared_ptr<AevaPublisherNode>>& publishers);

/// \brief Callback function that receives the IMU wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void IMUCallback(const api::ROS2IMUWrapper& wrapper,
                 std::vector<std::shared_ptr<AevaPublisherNode>>& publishers);

/// \brief Callback function that receives the Health wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void HealthCallback(const api::ROS2HealthWrapper& wrapper,
                    std::vector<std::shared_ptr<AevaPublisherNode>>& publishers);

/// \brief Callback function that receives the NavigationSolution wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void NavigationSolutionCallback(const api::ROS2NavigationSolutionWrapper& wrapper,
                                std::vector<std::shared_ptr<AevaPublisherNode>>& publishers);

/// \brief Callback function that receives the ObjectList wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
/// \param sensor_fusion_enabled Indicates if sensor fusion is enabled in the Aeva API.
void ObjectListCallback(const api::ROS2ObjectListWrapper& wrapper,
                        std::vector<std::shared_ptr<AevaPublisherNode>>& publishers,
                        bool sensor_fusion_enabled = false);

/// \brief Callback function that receives the SensorFusionObjectList wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
/// \param sensor_fusion_enabled Indicates if sensor fusion is enabled in the Aeva API.
void SensorFusionObjectListCallback(const api::ROS2ObjectListWrapper& wrapper,
                                    std::vector<std::shared_ptr<AevaPublisherNode>>& publishers,
                                    bool sensor_fusion_enabled = false);

/// \brief Callback function that receives the SPCPointGroup wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void SPCPointGroupCallback(const api::ROS2RawDataWrapper& wrapper,
                           std::vector<std::shared_ptr<AevaPublisherNode>>& publishers);

/// \brief Callback function that receives the CPCPointGroup wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \note Only supported for Atlas.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void CPCPointGroupCallback(const api::ROS2RawDataWrapper& wrapper,
                           std::vector<std::shared_ptr<AevaPublisherNode>>& publishers);

/// \brief Callback function that receives the RawPacket wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void RawPacketCallback(const api::ROS2RawDataWrapper& wrapper,
                       std::vector<std::shared_ptr<AevaPublisherNode>>& publishers);

/// \brief Callback function that receives the Reconfig wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void ReconfigCallback(const api::ROS2RawDataWrapper& wrapper,
                      std::vector<std::shared_ptr<AevaPublisherNode>>& publishers);

/// \brief Callback function that receives the Telemetry wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void TelemetryCallback(const api::ROS2TelemetryWrapper& wrapper,
                       std::vector<std::shared_ptr<AevaPublisherNode>>& publishers);

/// \brief Callback function that receives the Config wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void ConfigCallback(const api::ROS2RawDataWrapper& wrapper,
                    std::vector<std::shared_ptr<AevaPublisherNode>>& publishers);

/// \brief Callback function that receives the Log wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void LogCallback(const api::ROS2RawDataWrapper& wrapper,
                 std::vector<std::shared_ptr<AevaPublisherNode>>& publishers);

/// \brief Callback function that receives the Metadata wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS2 publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void MetadataCallback(const std_msgs::msg::String& metadata,
                      std::vector<std::shared_ptr<AevaPublisherNode>>& publishers);

}  // namespace ros2_examples
}  // namespace api
}  // namespace aeva
