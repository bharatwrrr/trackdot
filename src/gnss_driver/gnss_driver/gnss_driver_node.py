#!/usr/bin/env python3
"""
gnss_driver_node.py

Reads the CSS Electronics CANmod.GPS directly over USB using python-can's
csscan_serial interface.  No MQTT, no slcand, no kernel module required.

Signal decoding is fully handled by cantools reading your .dbc file —
you do not specify bit positions or factors anywhere in this code.

Topics published
────────────────────────────────────────────────────────────────────────
  gnss/status    → std_msgs/Bool                    (fix valid flag)
  gnss/time      → sensor_msgs/TimeReference        (GNSS epoch)
  gnss/position  → sensor_msgs/NavSatFix            (lat, lon, accuracy)
  gnss/altitude  → std_msgs/Float64                 (metres MSL)
  gnss/odometer  → std_msgs/Float64                 (total distance, metres)
  gnss/speed     → geometry_msgs/TwistStamped       (linear.x m/s)
"""

import math
import threading
import time
from pathlib import Path

import can
import cantools
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference, Imu
from std_msgs.msg import Bool, Float64

# ── Message names that must exist in the DBC ─────────────────────────────────
WANTED_MESSAGES = {
    "GnssStatus",
    "GnssTime",
    "GnssPosition",
    "GnssAltitude",
    "GnssOdo",
    "GnssSpeed",
    "GnssImu"
}


class GnssDriverNode(Node):

    def __init__(self) -> None:
        super().__init__("gnss_driver")

        # ── Declare parameters ────────────────────────────────────────────────
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("dbc_path",    "")       # required — no default
        self.declare_parameter("frame_id",    "gnss")

        port       = self.get_parameter("serial_port").value
        dbc_path   = self.get_parameter("dbc_path").value
        self._fid  = self.get_parameter("frame_id").value

        if not dbc_path:
            raise RuntimeError(
                "Parameter 'dbc_path' is required. "
                "Set it to the absolute path of your .dbc file in gnss_params.yaml."
            )
        if not Path(dbc_path).is_file():
            raise RuntimeError(f"DBC file not found: {dbc_path}")

        # ── Load DBC and build frame-ID lookup ────────────────────────────────
        self.get_logger().info(f"Loading DBC: {dbc_path}")
        db = cantools.database.load_file(dbc_path)

        # Map CAN frame_id → cantools Message object for fast lookup in the
        # hot path.  Only index the six messages we actually publish.
        self._msg_by_id: dict[int, cantools.database.Message] = {}
        for msg in db.messages:
            if msg.name in WANTED_MESSAGES:
                self._msg_by_id[msg.frame_id] = msg
                self.get_logger().info(
                    f"  registered  0x{msg.frame_id:08X}  →  {msg.name}"
                )

        missing = WANTED_MESSAGES - {m.name for m in self._msg_by_id.values()}
        if missing:
            self.get_logger().warn(
                f"The following messages were not found in the DBC and will "
                f"not be published: {', '.join(sorted(missing))}"
            )

        # ── QoS — best-effort, keep last 1 (sensor data) ─────────────────────
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self._pub_status   = self.create_publisher(Bool,          "gnss/status",   qos)
        self._pub_time     = self.create_publisher(TimeReference, "gnss/time",     qos)
        self._pub_position = self.create_publisher(NavSatFix,     "gnss/position", qos)
        self._pub_altitude = self.create_publisher(Float64,       "gnss/altitude", qos)
        self._pub_odometer = self.create_publisher(Float64,       "gnss/odometer", qos)
        self._pub_speed    = self.create_publisher(TwistStamped,  "gnss/speed",    qos)
        self._pub_imu = self.create_publisher(Imu, "gnss/imu", qos)

        # ── Open CAN bus via python-can csscan_serial ─────────────────────────
        # python-can handles the CSS Electronics proprietary binary USB
        # protocol transparently.  No slcand or kernel module needed.
        self.get_logger().info(f"Opening csscan_serial on {port} ...")
        try:
            self._bus = can.Bus(interface="csscan_serial", channel=port)
        except Exception as exc:
            raise RuntimeError(
                f"Failed to open csscan_serial on {port}: {exc}\n"
                f"  • Is the CANmod.GPS plugged in?\n"
                f"  • Run 'ls /dev/ttyACM*' to find the correct port.\n"
                f"  • You may need: sudo usermod -aG dialout $USER  (then log out/in)"
            ) from exc

        # ── Dedicated reader thread ───────────────────────────────────────────
        self._running = True
        self._thread  = threading.Thread(
            target=self._read_loop,
            name="gnss_can_reader",
            daemon=True,          # dies automatically if the process exits
        )
        self._thread.start()

        self.get_logger().info("GNSS driver running.")

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self.get_logger().info("Shutting down GNSS driver ...")
        # Signal the thread to stop first, wait for it to exit cleanly,
        # then shut the bus down — avoids closing the fd mid-read.
        self._running = False
        self._thread.join(timeout=2.0)
        self._bus.shutdown()

    # ── CAN reader thread ─────────────────────────────────────────────────────

    def _read_loop(self) -> None:
        """
        Blocks on bus.recv() with a 100 ms timeout so _running is checked
        regularly.  When a frame arrives, decode and publish immediately.
        All python-can I/O releases the GIL, so rclpy.spin() on the main
        thread is unaffected.
        """
        while self._running:
            try:
                frame = self._bus.recv(timeout=0.1)
            except TypeError:
                # Bus was shut down mid-read — exit cleanly
                break
            except can.CanError as exc:
                self.get_logger().warn(
                    f"CAN read error: {exc}", throttle_duration_sec=5.0
                )
                time.sleep(0.05)
                continue

            if frame is None:
                continue                # recv() timed out — check _running

            db_msg = self._msg_by_id.get(frame.arbitration_id)
            if db_msg is None:
                continue                # not one of the six messages we want

            try:
                signals = db_msg.decode(frame.data, decode_choices=False)
            except Exception as exc:    # noqa: BLE001
                self.get_logger().warn(
                    f"Decode error [{db_msg.name}]: {exc}",
                    throttle_duration_sec=5.0,
                )
                continue

            if rclpy.ok() and self._running:
                self._dispatch(db_msg.name, signals)


    # ── Dispatcher ────────────────────────────────────────────────────────────

    def _dispatch(self, name: str, signals: dict) -> None:
        now = self.get_clock().now().to_msg()
        match name:
            case "GnssStatus":   self._handle_status(signals)
            case "GnssTime":     self._handle_time(signals, now)
            case "GnssPosition": self._handle_position(signals, now)
            case "GnssAltitude": self._handle_altitude(signals)
            case "GnssOdo":      self._handle_odometer(signals)
            case "GnssSpeed":    self._handle_speed(signals, now)
            case "GnssImu": self._handle_imu(signals, now)

    # ── Per-message handlers ──────────────────────────────────────────────────

    def _handle_status(self, s: dict) -> None:
        msg      = Bool()
        msg.data = bool(s.get("StatusValid", 0))
        self._pub_status.publish(msg)

    def _handle_time(self, s: dict, now: RosTime) -> None:
        epoch = float(s.get("Epoch", 0.0))      # Unix seconds (float)
        valid = bool(s.get("TimeValid", 0))

        sec  = int(epoch)
        nsec = int((epoch - sec) * 1_000_000_000)

        msg                 = TimeReference()
        msg.header.stamp    = now
        msg.header.frame_id = self._fid
        msg.source          = "gnss_valid" if valid else "gnss_invalid"
        msg.time_ref        = RosTime(sec=sec, nanosec=max(0, nsec))
        self._pub_time.publish(msg)

    def _handle_position(self, s: dict, now: RosTime) -> None:
        valid = bool(s.get("PositionValid", 0))
        lat   = float(s.get("Latitude",          0.0))
        lon   = float(s.get("Longitude",         0.0))
        acc   = float(s.get("PositionAccuracy",  0.0))   # metres CEP

        msg                 = NavSatFix()
        msg.header.stamp    = now
        msg.header.frame_id = self._fid

        msg.status.status  = (
            NavSatStatus.STATUS_FIX if valid else NavSatStatus.STATUS_NO_FIX
        )
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude  = lat
        msg.longitude = lon
        msg.altitude  = math.nan          # filled separately by GnssAltitude

        # Diagonal covariance from CEP accuracy (1-sigma approximation).
        # Vertical is typically ~3× worse than horizontal.
        var = acc * acc
        msg.position_covariance = [
            var, 0.0, 0.0,
            0.0, var, 0.0,
            0.0, 0.0, 9.0 * var,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._pub_position.publish(msg)

    def _handle_altitude(self, s: dict) -> None:
        # The device outputs -5400 m when no fix — suppress it.
        if not bool(s.get("AltitudeValid", 0)):
            return

        msg      = Float64()
        msg.data = float(s.get("Altitude", 0.0))   # metres, factor applied by cantools
        self._pub_altitude.publish(msg)

    def _handle_odometer(self, s: dict) -> None:
        msg      = Float64()
        msg.data = float(s.get("DistanceTotal", 0.0))   # metres
        self._pub_odometer.publish(msg)

    def _handle_speed(self, s: dict, now: RosTime) -> None:
        valid = bool(s.get("SpeedValid", 0))
        speed = float(s.get("Speed", 0.0))   # m/s — cantools applies DBC factor

        msg                 = TwistStamped()
        msg.header.stamp    = now
        msg.header.frame_id = self._fid
        msg.twist.linear.x  = speed if valid else 0.0
        # linear.y/z and angular are zero — GNSS gives scalar ground speed only
        self._pub_speed.publish(msg)


    def _handle_imu(self, s: dict, now: RosTime) -> None:
        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = self._fid

        # Linear Acceleration (m/s^2)
        msg.linear_acceleration.x = float(s.get("AccelerationX", 0.0))
        msg.linear_acceleration.y = float(s.get("AccelerationY", 0.0))
        msg.linear_acceleration.z = float(s.get("AccelerationZ", 0.0))

        # Angular Velocity (rad/s)
        msg.angular_velocity.x = float(s.get("AngularRateX", 0.0))
        msg.angular_velocity.y = float(s.get("AngularRateY", 0.0))
        msg.angular_velocity.z = float(s.get("AngularRateZ", 0.0))

        # Orientation is not provided by raw IMU; usually requires fusion (EKF)
        msg.orientation_covariance[0] = -1.0  # Indicates orientation not available

        self._pub_imu.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = GnssDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
