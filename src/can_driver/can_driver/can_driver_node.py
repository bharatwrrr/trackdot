#!/usr/bin/env python3

import can
import time
import rclpy
from rclpy.node import Node
from trackdot_msgs.msg import VehicleState

# ISO-TP / UDS constants for Ioniq 5 gateway
REQUEST_ID   = 0x7B3
RESPONSE_ID  = 0x7BB
FLOW_CTRL_ID = 0x7B3

KMH_TO_MPS = 1.0 / 3.6 #TODO: need to check if this is correct for Ioniq 5, or if it needs to be scaled differently


class CanDriverNode(Node):

    def __init__(self):
        super().__init__('can_driver_node')

        # Parameters
        self.declare_parameter('channel', '/dev/ttyACM0')
        self.declare_parameter('interface', 'csscan_serial')
        self.declare_parameter('poll_rate_hz', 20.0)
        self.declare_parameter('frame_id', 'base_link')

        channel      = self.get_parameter('channel').value
        interface    = self.get_parameter('interface').value
        poll_rate_hz = self.get_parameter('poll_rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value

        # Publisher
        self.pub = self.create_publisher(VehicleState, 'vehicle_state', 10)

        # CAN bus
        try:
            self.bus = can.Bus(interface=interface, channel=channel)
            self.get_logger().info(f'CAN bus opened: {interface} on {channel}')
        except Exception as e:
            self.get_logger().fatal(f'Failed to open CAN bus: {e}')
            raise

        # Poll timer
        period = 1.0 / poll_rate_hz
        self.timer = self.create_timer(period, self._poll_and_publish)

    def _send_flow_control(self) -> None:
        fc = can.Message(
            arbitration_id=FLOW_CTRL_ID,
            data=[0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            is_extended_id=False,
        )
        self.bus.send(fc)

    def _poll_speed_mps(self) -> float | None:
        """
        Sends a UDS 0x22 request for DID 0x0100 and reassembles the
        ISO-TP multi-frame response.  Returns speed in m/s, or None on
        timeout / parse failure.
        """
        req = can.Message(
            arbitration_id=REQUEST_ID,
            data=[0x03, 0x22, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
            is_extended_id=False,
        )
        try:
            self.bus.send(req)
        except can.CanError as e:
            self.get_logger().warn(f'CAN send error: {e}')
            return None

        frames: list[bytes] = []
        total_len = 0
        deadline = time.time() + 0.15  # 150 ms window

        while time.time() < deadline:
            msg = self.bus.recv(timeout=0.02)
            if not msg or msg.arbitration_id != RESPONSE_ID:
                continue

            frame_type = (msg.data[0] & 0xF0) >> 4

            if frame_type == 1:          # First Frame
                total_len = ((msg.data[0] & 0x0F) << 8) | msg.data[1]
                frames.append(bytes(msg.data[2:]))
                self._send_flow_control()

            elif frame_type == 2:        # Consecutive Frame
                frames.append(bytes(msg.data[1:]))

        if not frames:
            return None

        full_payload = b''.join(frames)[:total_len]

        # Positive UDS response: 0x62 + DID echo 0x01 0x00
        if len(full_payload) <= 29 or full_payload[0:3] != b'\x62\x01\x00':
            return None

        data_bytes = full_payload[3:]
        raw_speed_kmh = data_bytes[29]
        return raw_speed_kmh * KMH_TO_MPS

    def _poll_and_publish(self) -> None:
        speed_mps = self._poll_speed_mps()
        if speed_mps is None:
            return

        msg = VehicleState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.speed_mps       = speed_mps
        self.pub.publish(msg)

    def destroy_node(self) -> None:
        self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
