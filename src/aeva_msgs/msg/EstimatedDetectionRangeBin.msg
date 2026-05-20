# Copyright 2016-2025 Aeva, Inc. All rights reserved.
# 
# Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
# code for terms and conditions that govern your use of this software. Any use, reproduction,
# disclosure, or distribution of this software and related documentation outside the terms of the
# EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

# Contains identifying information for a region in FOV and estimated detection range for it

# Frame index, non-zero only for dithered patterns
uint8 frame_index

# Beam index specifies the beam which falls in the FOV region
uint8 beam_index

# Line index specifies the line which falls in the FOV region
uint16 line_index

# Elevation of the line which falls in the FOV region, in radians
float32 elevation_rad

# Estimated detection range in the FOV region, in meters
float32 estimated_detection_range_m
