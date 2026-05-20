# Copyright 2016-2025 Aeva, Inc. All rights reserved.
# 
# Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
# code for terms and conditions that govern your use of this software. Any use, reproduction,
# disclosure, or distribution of this software and related documentation outside the terms of the
# EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

# Message definition for raw data from the sensor.

std_msgs/Header header

# The header contains the acquisition timestamp of the message data.
# This is the timestamp when the message was last published from the sensor in nanoseconds.
uint64 publish_time

# The sequence ID of the incoming message.
uint16 sequence_id

# Version of the message, the same version as Some IP Header.
uint8 version

# The type of incoming message data (see Message Data Formats manual).
uint8 message_type

# The length of the message header and message data in bytes.
uint32 message_size

# Raw message data.
uint8[] data
