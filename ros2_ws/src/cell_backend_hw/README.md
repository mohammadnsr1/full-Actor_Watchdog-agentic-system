# cell_backend_hw (scaffold)

Hardware implementation of `cell_platform_interfaces` for the UR7e + OnRobot
RG6 cell on the UR AI Accelerator (Humble). Builds on Humble and Jazzy; runs
only where the UR driver runs.

| Backend | Wraps | Open items (`config/platforms/hardware.yaml`) |
|---|---|---|
| `HardwareMotion` | UR driver trajectory controller `FollowJointTrajectory` | `motion.action` / `motion.cancel_service` confirmed against the installed driver branch; status topics |
| `URScriptRG6Gripper` | OnRobot URCap over the driver's URScript topic | `gripper.urscript.grip_function`, `release_function`, `width_range_m`, `force_range_n`; no readings over ROS 2 |
| pose sources | `/camera1/poses`, `/camera2/poses` | FoundationPose pipelines do not exist yet; lookups return None |
| `OperatorScene` | terminal prompt; verify against camera2 poses | `scene.confirm` other than terminal |

Readings the platform does not provide are reported as `None` with source
`missing`. Commands that need a TODO value return `ok=False` naming the
value. Nothing here starts or stops the robot program.
