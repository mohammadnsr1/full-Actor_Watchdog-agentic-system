"""The cuRobo-backed planning service node (same services as the MoveIt node)."""

from __future__ import annotations

import sys
from pathlib import Path

from cell_planning_service.cell_config import load_cell_config, planner_section, repo_root

from cell_planning_curobo.venv import ensure_curobo_environment


def _resolve(spec: str, root: Path) -> Path:
    if ":" in spec and not spec.startswith("/"):
        package, rel = spec.split(":", 1)
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package)) / rel
    p = Path(spec).expanduser()
    return p if p.is_absolute() else root / p


def main(args: list[str] | None = None) -> int:
    # Config is read before the re-exec so the interpreter path is known.
    config, path = load_cell_config()
    if not config:
        print(f"cell_planning_curobo: config/cell.yaml not found (looked at {path})", file=sys.stderr)
        return 2
    planner = planner_section(config)
    if str(planner.get("backend", "")).strip().lower() != "curobo":
        print("cell_planning_curobo: planner.backend is not 'curobo'; refusing to start", file=sys.stderr)
        return 2
    curobo_cfg = dict(planner.get("curobo", {}) or {})
    try:
        ensure_curobo_environment(str(curobo_cfg.get("python", "")))
    except RuntimeError as exc:
        print(f"cell_planning_curobo: {exc}", file=sys.stderr)
        return 2

    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    from cell_planning_service.service_server import DEFAULT_SERVICE_NAMESPACE, PlanningServiceServer

    from cell_planning_curobo.curobo_backend import CuRoboBackend

    root = repo_root() or path.parent.parent
    rclpy.init(args=args)
    node = Node("cell_planning_service")
    node.declare_parameter("cell_config_path", "")
    group = ReentrantCallbackGroup()
    limits_text = None
    spec = str(curobo_cfg.get("joint_limits_file", ""))
    if spec:
        lp = _resolve(spec, root)
        limits_text = lp.read_text(encoding="utf-8") if lp.is_file() else None
    depth_layer = None
    try:
        tensor_args = None
        if bool(planner.get("depth_obstacles", False)):
            from curobo.types.base import TensorDeviceType  # type: ignore

            from cell_planning_curobo.depth_obstacles import DepthObstacleLayer

            tensor_args = TensorDeviceType()
            depth_layer = DepthObstacleLayer(node, dict(curobo_cfg.get("depth", {}) or {}), tensor_args, group)
            node.get_logger().warning("depth obstacle layer is ON (experimental)")
        backend = CuRoboBackend(
            node,
            defaults=curobo_cfg,
            repo_root=root,
            joint_limits_text=limits_text,
            planning_frame=str(curobo_cfg.get("base_link", "base_link")),
            callback_group=group,
            joint_state_topic=str(planner.get("joint_state_topic", "/joint_states")),
            depth_layer=depth_layer,
        )
    except RuntimeError as exc:
        print(f"cell_planning_curobo: {exc}", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 2
    namespace = str(planner.get("service_namespace", DEFAULT_SERVICE_NAMESPACE))
    PlanningServiceServer(node, backend, namespace=namespace, callback_group=group)
    node.get_logger().info(f"planning service up: backend=curobo namespace={namespace} config={path}")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
