# Copyright 2016-2025 Aeva, Inc. All rights reserved.
# 
# Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
# code for terms and conditions that govern your use of this software. Any use, reproduction,
# disclosure, or distribution of this software and related documentation outside the terms of the
# EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

# Message definition for Cartesian point cloud metadata.

## Enumerators ##

# Platform type enum values.
uint8 PLATFORM_UNKNOWN=0
uint8 PLATFORM_AERIES_1=1
uint8 PLATFORM_AERIES_2=2

# Range mode enum values.
uint8 RANGE_NORMAL=0
uint8 RANGE_LONG=1
uint8 RANGE_ULTRA_LONG=2
uint8 RANGE_IMPROVED_RESOLUTION=3

# Reference frame enum values.
uint8 REFERENCE_FRAME_UNKNOWN=0
uint8 REFERENCE_FRAME_WORLD=1
uint8 REFERENCE_FRAME_VEHICLE=2
uint8 REFERENCE_FRAME_SENSOR=3
uint8 REFERENCE_FRAME_CAMERA=10
uint8 REFERENCE_FRAME_IMU=11

#--------------------------------------------------------------------------------------------------#

std_msgs/Header header

# This is the timestamp when the message was last published from the Aeva API in nanoseconds.
uint64 publish_time

# The sequence ID of the point cloud message.
uint16 sequence_id

# Major version of the CPC message.
# Updated if the memory layout of the CPC message is changed (eg: addition of new fields).
uint8 major_version

# Minor version of the CPC message.
# Updated if the range of existing fields change or any reserved fields were revived for usage.
uint8 minor_version

#--------------------------------------------------------------------------------------------------#

# Additional metadata (for internal use).
uint64 metadata

# Index of a frame boundary point. If -1, the point cloud does not contain a frame boundary.
# If non-zero, it indicates points' index where the frame boundary is contained.
int32 frame_sync_index

# Used to determine how to interpret metadata (for internal use).
int8 lidar_data_version

# Platform that generated the point cloud.
uint8 platform

# The time period (nsec) between two points.
uint16 nano_seconds_per_index

# Reference frame for the point cloud data.
uint8 reference_frame

# Dither step index of the first point in frame. If there's a frame transition,
# (frame_sync_index_ > 0), the dither step needs to be incremented for next frame.
# In the case of frame_sync_index == 0 (frame transition at the first point),
# the dither step already contains the correct index.
uint8 dither_step

# When frame dither is enabled, the scan pattern is offset by small elevation angles at every frame.
# This determines the maximum value dither_step can take before wrapping around to 0.
uint8 max_dither_step

# Point cloud period in ns.
uint32 period_ns

# Bit map for boolean flags. (Bit 1: LSB, Bit 8: MSB)
# Bit 1: Indicates whether frame builder accumulated two frames.
#        This can be true for dithered patterns only.
# Bit 2: Indicates whether a high number of points were rejected by dynamic noise frame filter.
#        This can be true if the ego vehicle is passing through dusty/foggy conditions.
# Bits 3-8: Unset.
uint8 flags

#--------------------------------------------------------------------------------------------------#

## Parsed metadata fields ##

# The range mode that the sensor was set to.
uint8 range_mode

# The number of beams.
uint16 num_beams

# The number of lines per beam.
uint16 num_lines

# The number of peaks per point.
uint16 num_peaks

# Flag that specifies whether the line indices are absolute or beam-relative.
bool is_absolute_line_index
