"""Relay the Kit command server's lab-event journal (EB3) onto topics.lab_events
as std_msgs/String, one JSON object per event:

    {"seq", "kind": reset | restore | set, "objects": [...], "detail": {...},
     "sim_time_s", "wall_time_iso"}

A lab verb that changes the world (isaac/cell_command_server/lab.py) appends an
event; this node polls lab_events_poll over loopback at scene.contacts_poll_hz,
so the WatchDog (EB7) and the recorder see teleports as declared lab actions,
not as physics. Survives a server restart (the sequence starts over).
"""

from __future__ import annotations

import json
import sys

import rclpy
from rclpy.node import Node

from cell_backend_sim.scene import CommandServerClient
from cell_platform_interfaces.platform_config import load_platform_config


class LabEventsRelay(Node):
    def __init__(self) -> None:
        super().__init__("lab_events_relay")
        from rclpy.clock import Clock, ClockType
        from std_msgs.msg import String

        self.platform = load_platform_config()
        self._String = String
        self.client = CommandServerClient(str(self.platform.get_optional("scene", "host", default="127.0.0.1")),
                                          int(self.platform.get("scene", "port")), timeout_s=2.0)
        self.pub = self.create_publisher(String, self.platform.topic("lab_events"), 50)
        self.last_seq = 0
        rate = float(self.platform.get("scene", "contacts_poll_hz"))
        self.timer = self.create_timer(1.0 / rate, self._poll, clock=Clock(clock_type=ClockType.SYSTEM_TIME))
        self.get_logger().info(f"relaying lab events from {self.client.host}:{self.client.port} to {self.pub.topic_name}")

    def _poll(self) -> None:
        resp = self.client.call("lab_events_poll", since_seq=self.last_seq)
        if not resp.get("ok"):
            self.get_logger().warning(f"waiting for the command server: lab_events_poll: {resp.get('error')}",
                                      throttle_duration_sec=10.0)
            return
        if int(resp.get("last_seq", 0)) < self.last_seq:      # server restarted
            self.last_seq = 0
            return
        for e in resp.get("events", []):
            self.last_seq = max(self.last_seq, int(e.get("seq", 0)))
            self.pub.publish(self._String(data=json.dumps(e, sort_keys=True)))


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = LabEventsRelay()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f"lab_events_relay: {exc}", file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
