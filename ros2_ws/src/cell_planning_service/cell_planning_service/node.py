"""The MoveIt-backed planning service node.

Reads config/cell.yaml (planner section), refuses to start unless
planner.backend is "moveit" (the cuRobo node is a different executable in
cell_planning_curobo), and serves the cell_planner_msgs services under
planner.service_namespace.

Parameters:
    cell_config_path   explicit path to cell.yaml (else CELL_CONFIG_PATH,
                       else the repo-root walk-up in cell_config.py)
    joint_limits_path  explicit path to a joint_limits.yaml; default is the
                       `planner.moveit.joint_limits_file` value, resolved as
                       `<package>:<relative path>` through ament, or an
                       absolute path
"""

from __future__ import annotations

import sys
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cell_planning_service.cell_config import load_cell_config, planner_section
from cell_planning_service.moveit_backend import MoveItBackend
from cell_planning_service.service_server import PlanningServiceServer


def resolve_package_path(spec: str) -> Path:
    """`<package>:<relative>` via ament share, or an absolute/relative path."""
    if ":" in spec and not spec.startswith("/"):
        package, rel = spec.split(":", 1)
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package)) / rel
    return Path(spec).expanduser()


class PlanningServiceNode(Node):
    def __init__(self) -> None:
        super().__init__("cell_planning_service")
        self.declare_parameter("cell_config_path", "")
        self.declare_parameter("joint_limits_path", "")
        explicit = str(self.get_parameter("cell_config_path").value)
        config, path = load_cell_config(explicit or None)
        if not config:
            raise RuntimeError(
                f"config/cell.yaml not found (looked at {path}); set CELL_CONFIG_PATH or cell_config_path"
            )
        planner = planner_section(config)
        backend_name = str(planner.get("backend", "")).strip().lower()
        if backend_name != "moveit":
            raise RuntimeError(
                f"planner.backend is {backend_name!r}; this executable serves the moveit backend only. "
                "Launch through cell_planning_service/launch/planning_service.launch.py, which picks the node."
            )
        moveit_defaults = dict(planner.get("moveit", {}) or {})
        limits_text = None
        limits_spec = str(self.get_parameter("joint_limits_path").value) or str(
            moveit_defaults.get("joint_limits_file", "")
        )
        if limits_spec:
            limits_path = resolve_package_path(limits_spec)
            if limits_path.is_file():
                limits_text = limits_path.read_text(encoding="utf-8")
            else:
                self.get_logger().warning(f"joint limits file not found: {limits_path}")
        # Names come from the platform config (config/platforms/<platform>.yaml);
        # planner.service_namespace / planner.joint_state_topic in cell.yaml
        # override them when set.
        from cell_platform_interfaces.platform_config import load_platform_config

        platform = load_platform_config(repo_root=path.parent.parent if path else None)
        namespace = str(planner.get("service_namespace") or platform.service("planning_namespace"))
        joint_state_topic = str(planner.get("joint_state_topic") or platform.topic("joint_states"))
        moveit_defaults["services"] = {
            key: platform.service(key)
            for key in ("plan_kinematic_path", "compute_cartesian_path", "check_state_validity",
                        "apply_planning_scene", "sync_scene_objects")
        }
        group = ReentrantCallbackGroup()
        self.backend = MoveItBackend(
            self,
            defaults=moveit_defaults,
            joint_limits_text=limits_text,
            callback_group=group,
            joint_state_topic=joint_state_topic,
        )
        self.server = PlanningServiceServer(
            self, self.backend, namespace=namespace, callback_group=group,
            logger=_RclpyLoggerAdapter(self.get_logger()),
        )
        self.get_logger().info(
            f"planning service up: backend=moveit namespace={namespace} config={path}"
        )


class _RclpyLoggerAdapter:
    """Minimal logging.Logger-like wrapper over an rclpy logger."""

    def __init__(self, logger) -> None:
        self._l = logger

    def warning(self, msg, *args) -> None:
        self._l.warning(msg % args if args else str(msg))

    def info(self, msg, *args) -> None:
        self._l.info(msg % args if args else str(msg))

    def exception(self, msg, *args) -> None:
        import traceback

        self._l.error((msg % args if args else str(msg)) + "\n" + traceback.format_exc())


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = PlanningServiceNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f"cell_planning_service: {exc}", file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.backend.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
