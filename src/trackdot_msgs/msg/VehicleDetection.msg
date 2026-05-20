# trackdot_msgs/msg/VehicleDetection.msg
#
# A single 2D camera detection from a DeepStream PGIE+SGIE pipeline,
# with optional in-pipeline tracker association.
#
# All bbox coordinates are in pixels of the streammux output resolution
# (see streammux.width / streammux.height in the camera config). Downstream
# consumers (lidar_cam_fusion) must scale if they project from a different
# image resolution.

# ----- Source identification -----
string  camera_id            # e.g. "cam_left", "cam_right" — matches TF frame
uint32  pgie_unique_id       # nvinfer gie-unique-id of the PGIE that produced this
uint32  sgie_unique_id       # nvinfer gie-unique-id of the SGIE (0 if no classifier ran)

# ----- Tracker association (from nvtracker) -----
# track_id is stable across frames within a single camera stream.
# It is NOT cross-camera — fusion/tracker upstream packages do that.
# Value is UINT64_MAX when the object was detected but not yet tracked.
uint64  track_id

# ----- Primary detector output (PGIE) -----
uint32  class_id             # PGIE class index (e.g. 2 = Car for trafficcamnet)
string  class_label          # Resolved label string, optional
float32 pgie_confidence      # [0.0, 1.0]

# ----- 2D bounding box in image pixels -----
# Top-left origin, x to the right, y down. Pixel-space, not normalized.
float32 bbox_x               # left
float32 bbox_y               # top
float32 bbox_width
float32 bbox_height

# ----- Secondary classifier output (SGIE) -----
# Empty string + 0.0 confidence when no classifier ran for this object
# (e.g. classifier filtered by min size, async mode skipped, or class_id
# not in operate-on-class-ids).
string  sgie_label           # e.g. "compact_cars", "standard_pickup_trucks"
float32 sgie_confidence      # [0.0, 1.0]
