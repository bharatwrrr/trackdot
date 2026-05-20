// Copyright 2016-2025 Aeva, Inc. All rights reserved.
// 
// Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
// code for terms and conditions that govern your use of this software. Any use, reproduction,
// disclosure, or distribution of this software and related documentation outside the terms of the
// EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

// This program demonstrates how to use ROS2 build of the Aeva API to publish ROS2 messages from the
// sensor. Unlike the callbacks with the standard Aeva API, this version of the API directly
// provides the converted ROS2 types.

#include <unistd.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "aeva/api/AevaAPI.h"

#include "AevaAPICallbacks.h"
#include "AevaPublisherNode.h"

using namespace aeva;
using namespace aeva::api::ros2_examples;

namespace {

// Time interval to poll the events (set to 2x the highest frequency stream from the API).
// Point clouds are published at 10 Hz and IMU is published 100 Hz (highest frequency stream).
constexpr int64_t kPollIntervalMilliseconds{5};
constexpr int64_t kReconfigPollIntervalMilliseconds{10000};

}  // namespace

std::string CodeToString(api::ConfigReturn code) {
  if (code == api::ConfigReturn::OK) {
    return "OK";
  } else if (code == api::ConfigReturn::FAIL) {
    return "FAIL";
  } else if (code == api::ConfigReturn::TIMEOUT) {
    return "TIMEOUT";
  } else if (code == api::ConfigReturn::INVALID_SENSOR) {
    return "INVALID_SENSOR";
  } else {
    return "UNKNOWN";
  }
}

template <typename ConfigValueT>
void SetConfig(api::AevaAPI& api, const std::string& sensor_id, const std::string& node,
               const std::string& key, const ConfigValueT& value) {
  auto code = api.ConfigureValue(sensor_id, node, key, value);
  if (code != api::ConfigReturn::OK) {
    std::cerr << "Failed to set config: " << node << "." << key << " = " << value
              << " [status: " << CodeToString(code) << "]" << std::endl;
  }
}

int main(int argc, char** argv) {
  auto logger = rclcpp::get_logger("main");

  // Takes the sensor information as input
  if (argc < 3) {
    RCLCPP_FATAL_STREAM(logger,
                        "Not enough arguments. Please provide the sensor information in the given "
                        "sequence: SENSOR_IP_0 SENSOR_NAME_0 [... SENSOR_IP_N SENSOR_NAME_N]");
    return EXIT_FAILURE;
  }

  // Using a unique node name so multiple instances can be run.
  rclcpp::init(argc, argv);
  std::srand(std::time(0));
  std::ostringstream runner_name;
  runner_name << "aeva_runner_" << std::to_string(std::rand() % 256);

  // Initialize a dummy node to parse the node parameters.
  auto node = rclcpp::Node::make_shared(runner_name.str());
  RCLCPP_INFO_STREAM(node->get_logger(), "Node name: " << runner_name.str());

  // Parse node parameters.
  // Set --ros-args -p KEY:=VALUE from command line AFTER the positional arguments.

  // Aeva raw data messages are required to open the bag file using Aeviz.
  node->declare_parameter<bool>("publish_raw_topics", true);
  bool publish_raw_topics = node->get_parameter("publish_raw_topics").as_bool();
  RCLCPP_INFO_STREAM(node->get_logger(),
                     "Parsed publish_raw_topics: " << std::boolalpha << publish_raw_topics);

  // Enable publishing output in native ROS formats.
  node->declare_parameter<bool>("publish_ros_topics", true);
  bool publish_ros_topics = node->get_parameter("publish_ros_topics").as_bool();
  RCLCPP_INFO_STREAM(node->get_logger(),
                     "Parsed publish_ros_topics: " << std::boolalpha << publish_ros_topics);

  // Pass-through the raw sensor data from the sensor to the callbacks, bypass all API algorithms.
  node->declare_parameter<bool>("bypass_api", false);
  bool bypass_api = node->get_parameter("bypass_api").as_bool();
  RCLCPP_INFO_STREAM(node->get_logger(), "Parsed bypass_api: " << std::boolalpha << bypass_api);

  // This is used to offset the sensor indices; it's needed for running and recording messages from
  // multiple instances of this app.
  node->declare_parameter<int>("sensor_index_offset", 0);
  int sensor_index_offset = node->get_parameter("sensor_index_offset").as_int();
  RCLCPP_INFO_STREAM(node->get_logger(), "Parsed sensor_index_offset: " << sensor_index_offset);

  // The sensor ID is used to update the topic and frame ID namespaces, which is useful when
  // multiple instances of the ROS publisher node are instantiated.
  std::vector<std::string> sensor_ids;
  std::vector<std::string> sensor_urls;
  for (int i = 1; i < argc; i += 2) {
    if (std::string(argv[i]) == "--ros-args" || std::string(argv[i + 1]) == "--ros-args") {
      break;  // Stop parsing when --ros-args is encountered.
    }
    sensor_urls.push_back(std::string(argv[i]));

    // Sensor IDs must conform to ROS naming scheme where underscore is the sole
    // special character allowed
    std::string sensor_id{std::string(argv[i + 1])};
    for (char& ch : sensor_id) {
      if (!std::isalnum(ch)) {
        ch = '_';
      }
    }
    sensor_ids.push_back(sensor_id);
  }
  RCLCPP_INFO_STREAM(logger, "Parsed " << sensor_ids.size() << " sensor(s) information");

  // Instantiate the ROS publishers.
  std::vector<std::shared_ptr<AevaPublisherNode>> publishers;
  std::size_t sensor_index = sensor_index_offset;
  for (const auto& sensor_id : sensor_ids) {
    auto node_name = "aeva_publisher_" + sensor_id;
    publishers.emplace_back(std::make_shared<AevaPublisherNode>(
        node_name, sensor_id, sensor_index++, sensor_index_offset, logger, publish_raw_topics,
        publish_ros_topics));
  }

  api::AevaAPI api;

  // Connect to the Atlas sensors.
  std::vector<api::Sensor> sensors;
  for (std::size_t i = 0; i < sensor_ids.size(); ++i) {
    api::Sensor sensor;
    sensor.id_ = sensor_ids[i];
    sensor.url_ = sensor_urls[i];
    sensor.platform_.type_ = api::Sensor::Platform::Type::ATLAS;
    sensor.platform_.configuration_ = api::Sensor::Platform::Configuration::STANDARD;
    sensor.protocol_ = api::Sensor::Protocol::UDP_UNICAST;
    auto processing_mode =
        bypass_api ? api::ProcessingMode::PASSTHROUGH : api::ProcessingMode::DEFAULT;
    RCLCPP_INFO_STREAM(logger, "Connecting to " << sensor_ids[i] << " at " << sensor_urls[i]);
    if (!api.Connect(sensor, processing_mode)) {
      RCLCPP_FATAL_STREAM(
          logger, "Failed to connect to sensor: " << sensor_ids[i] << " at " << sensor_urls[i]);
      return EXIT_FAILURE;
    }
    sensors.push_back(sensor);
  }

  // Bind callbacks with the publishers.
  auto calibration_callback =
      std::bind(CalibrationCallback, std::placeholders::_1, std::ref(publishers));
  auto config_callback = std::bind(ConfigCallback, std::placeholders::_1, std::ref(publishers));
  auto cpc_pointgroup_callback =
      std::bind(CPCPointGroupCallback, std::placeholders::_1, std::ref(publishers));
  auto health_callback = std::bind(HealthCallback, std::placeholders::_1, std::ref(publishers));
  auto imu_callback = std::bind(IMUCallback, std::placeholders::_1, std::ref(publishers));
  auto log_callback = std::bind(LogCallback, std::placeholders::_1, std::ref(publishers));
  auto metadata_callback = std::bind(MetadataCallback, std::placeholders::_1, std::ref(publishers));
  auto object_list_callback =
      std::bind(ObjectListCallback, std::placeholders::_1, std::ref(publishers), false);
  auto odometry_callback = std::bind(OdometryCallback, std::placeholders::_1, std::ref(publishers));
  auto point_cloud_callback =
      std::bind(PointCloudCallback, std::placeholders::_1, std::ref(publishers));
  auto reconfig_callback = std::bind(ReconfigCallback, std::placeholders::_1, std::ref(publishers));
  auto raw_packet_callback =
      std::bind(RawPacketCallback, std::placeholders::_1, std::ref(publishers));
  auto telemetry_callback =
      std::bind(TelemetryCallback, std::placeholders::_1, std::ref(publishers));

  // Register callbacks.
  api.RegisterCalibrationMessageCallback(calibration_callback);
  api.RegisterHealthMessageCallback(health_callback);
  api.RegisterIMUMessageCallback(imu_callback);
  api.RegisterObjectListMessageCallback(object_list_callback);
  api.RegisterOdometryMessageCallback(api::kOdometrySensorDataStreamId, odometry_callback);
  api.RegisterPointCloudMessageCallback(api::kPointCloudDataStreamId, point_cloud_callback);

  if (publish_raw_topics) {
    api.RegisterConfigMessageCallback(config_callback);
    api.RegisterLogMessageCallback(log_callback);
    api.RegisterMetadataMessageCallback(metadata_callback);
    api.RegisterNodeConfigurationMessageCallback(reconfig_callback);
    api.RegisterRawCartesianPointGroupMessageCallback(cpc_pointgroup_callback);
    api.RegisterRawPacketMessageCallback(raw_packet_callback);
    api.RegisterTelemetryMessageCallback(telemetry_callback);
  }

  // Subscribe and poll sensors.
  for (const auto& sensor : sensors) {
    api.Subscribe(sensor.id_, api::kCalibrationDataStreamId);
    api.Subscribe(sensor.id_, api::kHealthDataStreamId);
    api.Subscribe(sensor.id_, api::kIMUDataStreamId);
    api.Subscribe(sensor.id_, api::kObjectListDataStreamId);
    api.Subscribe(sensor.id_, api::kOdometrySensorDataStreamId);
    api.Subscribe(sensor.id_, api::kPointCloudDataStreamId);

    if (publish_raw_topics) {
      api.Subscribe(sensor.id_, api::kConfigDataStreamId);
      api.Subscribe(sensor.id_, api::kLogDataStreamId);
      api.Subscribe(sensor.id_, api::kRawCpcPointGroupDataStreamId);
      api.Subscribe(sensor.id_, api::kRawPacketDataStreamId);
      api.Subscribe(sensor.id_, api::kReconfigDataStreamId);
      api.Subscribe(sensor.id_, api::kTelemetryDataStreamId);
    }
  }

  std::size_t reconfig_poll_counter{0};
  while (rclcpp::ok()) {
    api.PollEvents();
    usleep(kPollIntervalMilliseconds * 1000);

    // Periodically poll the reconfig manifest. Requesting a reconfig manifest will cause all nodes
    // of the connected sensors to send a message containing the state of their reconfigs. These
    // messages can be recorded and used during playback.
    if (reconfig_poll_counter++ > kReconfigPollIntervalMilliseconds / kPollIntervalMilliseconds) {
      api.RequestReconfigManifest();
      reconfig_poll_counter = 0;
    }
  }

  // Unsubscribe and disconnect.
  api.UnsubscribeAll(api::kCalibrationDataStreamId);
  api.UnsubscribeAll(api::kConfigDataStreamId);
  api.UnsubscribeAll(api::kHealthDataStreamId);
  api.UnsubscribeAll(api::kIMUDataStreamId);
  api.UnsubscribeAll(api::kLogDataStreamId);
  api.UnsubscribeAll(api::kObjectListDataStreamId);
  api.UnsubscribeAll(api::kOdometrySensorDataStreamId);
  api.UnsubscribeAll(api::kPointCloudDataStreamId);
  api.UnsubscribeAll(api::kRawCpcPointGroupDataStreamId);
  api.UnsubscribeAll(api::kRawPacketDataStreamId);
  api.UnsubscribeAll(api::kReconfigDataStreamId);
  api.UnsubscribeAll(api::kSensorFusionObjectListDataStreamId);
  api.UnsubscribeAll(api::kTelemetryDataStreamId);

  api.DisconnectAll();

  return EXIT_SUCCESS;
}
