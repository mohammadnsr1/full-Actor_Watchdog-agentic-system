"""Relay the Kit command server's finger-pad contact reports onto the ROS
graph as cell_planner_msgs/PadContact on topics.pad_contacts.

Kit's Python cannot publish ROS 2 messages for the system distro, so the
command server keeps a short ring of contact events (contacts_enable,
contacts_poll) and this node polls it over loopback at scene.contacts_poll_hz.
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node

from cell_backend_sim.scene import CommandServerClient
from cell_platform_interfaces.platform_config import load_platform_config


class PadContactsRelay(Node):
    def __init__(self) -> None:
        super().__init__("pad_contacts_relay")
        from cell_planner_msgs.msg import PadContact

        self.platform = load_platform_config()
        self._Msg = PadContact
        self.client = CommandServerClient(str(self.platform.get_optional("scene", "host", default="127.0.0.1")), int(self.platform.get("scene", "port")), timeout_s=2.0)
        self.pub = self.create_publisher(PadContact, self.platform.topic("pad_contacts"), 50)
        self.last_seq = 0
        rate = float(self.platform.get("scene", "contacts_poll_hz"))
        enabled = self.client.call("contacts_enable", enable=True)
        if not enabled.get("ok"):
            raise RuntimeError(f"contacts_enable failed: {enabled.get('error')}")
        from rclpy.clock import Clock, ClockType

        self.timer = self.create_timer(1.0 / rate, self._poll, clock=Clock(clock_type=ClockType.SYSTEM_TIME))
        self.get_logger().info(f"relaying pad contacts from {self.client.host}:{self.client.port} to {self.pub.topic_name} (pads {enabled.get('pads')})")

    def _poll(self) -> None:
        resp = self.client.call("contacts_poll", since_seq=self.last_seq)
        if not resp.get("ok"):
            self.get_logger().warning(f"contacts_poll: {resp.get('error')}", throttle_duration_sec=5.0)
            return
        for e in resp.get("events", []):
            self.last_seq = max(self.last_seq, int(e.get("seq", 0)))
            if e.get("event") == "error":
                self.get_logger().warning(f"contact tracker: {e.get('error')}", throttle_duration_sec=5.0)
                continue
            msg = self._Msg()
            msg.wall_time_iso = str(e.get("wall_time_iso", ""))
            msg.sim_time = float(e.get("sim_time_s", 0.0))
            msg.pad = str(e.get("pad", ""))
            msg.body = str(e.get("body", ""))
            msg.event = str(e.get("event", ""))
            msg.impulse = float(e.get("impulse", 0.0) or 0.0)
            msg.shape_pairs = int(e.get("shape_pairs", 0) or 0)
            msg.pad_frame = str(e.get("pad_frame", "") or "")
            pw, pl = e.get("point_world"), e.get("point_in_pad_frame")
            msg.has_point = pw is not None
            if pw is not None:
                msg.point_world.x, msg.point_world.y, msg.point_world.z = (float(v) for v in pw)
            if pl is not None:
                msg.point_in_pad_frame.x, msg.point_in_pad_frame.y, msg.point_in_pad_frame.z = (float(v) for v in pl)
            d = e.get("distance_m")
            msg.distance_to_pad_frame_m = float(d) if d is not None else -1.0
            msg.on_pad_surface = bool(e.get("on_pad_surface", False))
            self.pub.publish(msg)


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = PadContactsRelay()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f"pad_contacts_relay: {exc}", file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
