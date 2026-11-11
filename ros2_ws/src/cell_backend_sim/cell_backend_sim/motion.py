"""Sim motion backend: the Isaac trajectory bridge's FollowJointTrajectory server.

Nothing sim-specific beyond the names in platforms/sim.yaml. The bridge
accepts cancels and publishes no hold, so a cancel leaves the arm at the
last setpoint already sent (Mohammad's choice, BUILD_PLAN v1 q6).
"""

from __future__ import annotations

from cell_platform_interfaces.fjt_motion import FollowJointTrajectoryMotion


class SimMotion(FollowJointTrajectoryMotion):
    pass
