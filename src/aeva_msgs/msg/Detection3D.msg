# Copyright 2016-2025 Aeva, Inc. All rights reserved.
# 
# Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
# code for terms and conditions that govern your use of this software. Any use, reproduction,
# disclosure, or distribution of this software and related documentation outside the terms of the
# EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

# Defines a 3D detection result.

int32 UNKNOWN=0

# Object classes.
int32 CAR=1
int32 TRUCK=2
int32 BUS=3
int32 MOTORBIKE=4
int32 TRAIN=5
int32 PEDESTRIAN=6
int32 BICYCLE=7
int32 VEHICLE=8

# Track status values.
int32 TRACKED=1
int32 EXTRAPOLATED=2

std_msgs/Header header

# The unique numeric ID of object detected.
int64 id

# 3D bounding box surrounding the object.
vision_msgs/BoundingBox3D bbox

# Classification for this object.
int32 object_class

# By convention, these values should lie in the range [0-1].
float64 classification_score

# Track status for this object.
int32 track_status

# By convention, these values should lie in the range [0-1].
float64 track_confidence

# Linear velocity of the object.
geometry_msgs/Vector3 velocity
