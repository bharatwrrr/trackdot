# Copyright 2016-2025 Aeva, Inc. All rights reserved.
# 
# Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
# code for terms and conditions that govern your use of this software. Any use, reproduction,
# disclosure, or distribution of this software and related documentation outside the terms of the
# EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

# A list of 3D detections, for a multi-object 3D detector.

std_msgs/Header header

# A list of the detected proposals. A multi-proposal detector might generate
# this list with many candidate detections generated from a single input.
aeva_msgs/Detection3D[] detections
