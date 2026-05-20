#!/usr/bin/env python3
"""
viz_overlay.py
Subscribes to a CompressedImage and a VehicleDetectionArray, draws bounding
boxes + labels with OpenCV, and republishes as CompressedImage.

Topics (remappable):
  ~/image_in   — sensor_msgs/CompressedImage   (from camera_detector viz branch)
  ~/detections — trackdot_msgs/VehicleDetectionArray
  ~/image_out  — sensor_msgs/CompressedImage   (subscribe this in RViz)
"""

import threading
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage
from trackdot_msgs.msg import VehicleDetectionArray

# One colour per class_id (cycles if more classes than colours)
COLOURS = [
    (0,   255,  0),   # green
    (255,  80,  80),  # red
    (80,  80,  255),  # blue
    (255, 200,   0),  # yellow
    (0,   220, 220),  # cyan
    (220,   0, 220),  # magenta
]


class VizOverlay(Node):
    def __init__(self):
        super().__init__("viz_overlay")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._lock       = threading.Lock()
        self._detections = []   # list of VehicleDetection, refreshed each callback

        self._img_sub = self.create_subscription(
            CompressedImage,
            "~/image_in",
            self._image_cb,
            sensor_qos,
        )

        self._det_sub = self.create_subscription(
            VehicleDetectionArray,
            "~/detections",
            self._detection_cb,
            sensor_qos,
        )

        self._pub = self.create_publisher(
            CompressedImage,
            "~/image_out",
            sensor_qos,
        )

        self.get_logger().info("viz_overlay ready")

    # ── Detection callback ────────────────────────────────────────────────────
    def _detection_cb(self, msg: VehicleDetectionArray):
        with self._lock:
            self._detections = msg.detections

    # ── Image callback ────────────────────────────────────────────────────────
    def _image_cb(self, msg: CompressedImage):
        # Decode JPEG
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        h, w = frame.shape[:2]

        # Grab latest detections under lock
        with self._lock:
            dets = list(self._detections)

        for det in dets:
            # camera_detector publishes coords in full-res space (1920×1200).
            # The viz image is downscaled (default 960×540), so scale bbox.
            sx = w / 1920.0
            sy = h / 1200.0

            x1 = int(det.bbox_x      * sx)
            y1 = int(det.bbox_y      * sy)
            x2 = int((det.bbox_x + det.bbox_width)  * sx)
            y2 = int((det.bbox_y + det.bbox_height) * sy)

            colour = COLOURS[det.class_id % len(COLOURS)]

            # Bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)

            # Label: sgie class if available, otherwise pgie class_id
            label = det.sgie_label if det.sgie_label else f"cls:{det.class_id}"
            conf  = det.sgie_confidence if det.sgie_label else det.pgie_confidence
            text  = f"{label} {conf:.2f}  id:{det.track_id}"

            # Background rectangle for readability
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame,
                          (x1, max(y1 - th - 6, 0)),
                          (x1 + tw + 4, y1),
                          colour, -1)
            cv2.putText(frame, text,
                        (x1 + 2, max(y1 - 4, th)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                       (0, 0, 0), 1, cv2.LINE_AA)

        # Re-encode as JPEG and publish
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return

        out              = CompressedImage()
        out.header       = msg.header
        out.format       = "jpeg"
        out.data         = buf.tobytes()
        self._pub.publish(out)


def main():
    rclpy.init()
    node = VizOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main() 
