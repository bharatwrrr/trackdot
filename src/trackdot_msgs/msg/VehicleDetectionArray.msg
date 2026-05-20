# trackdot_msgs/msg/VehicleDetectionArray.msg
#
# All detections from a single camera frame.
# header.stamp is the camera capture timestamp (PTP-synced via PPS).
# header.frame_id is the camera optical frame (e.g. "cam_left_optical").

std_msgs/Header                    header
trackdot_msgs/VehicleDetection[]   detections
