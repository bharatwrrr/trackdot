# Copyright 2016-2025 Aeva, Inc. All rights reserved.
# 
# Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
# code for terms and conditions that govern your use of this software. Any use, reproduction,
# disclosure, or distribution of this software and related documentation outside the terms of the
# EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

# Contains estimated detection range for all regions in the FOV
# Estimated detection range corresponds to the maximum range at which a
# 10% reflectivity target is detected with 80% probability of detection

std_msgs/Header header

# Sensor platform used when the message was generated
# Estimated detection range is useful for AERIES_2 platform only
uint8 PLATFORM_UNKNOWN = 0
uint8 PLATFORM_AERIES_2 = 1
uint8 platform

# Azimuth limits and resolution
float32 min_azimuth_rad
float32 max_azimuth_rad
uint32 num_az_steps

# Vector containing estimated detection range for all regions of the FOV
# fov_region_idx = frame_idx * num_beams * num_lines * num_az_steps +
#                  beam_idx * num_lines * num_az_steps +
#                  line_idx * num_az_steps +
#                  az_idx
EstimatedDetectionRangeBin[] estimated_detection_range_bins
