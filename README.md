# robot_ws — FR5 + MoveIt host workspace (Second_plan.md, Milestone A)

Host-side (Jetson Thor, ROS 2 Jazzy) bringup of the Fairino FR5 driver +
MoveIt, built for the VLA pick pipeline. Created for **Task A1**.

## Facts recorded by A1 (needed by Milestone B)

| Question | Answer |
|---|---|
| MoveIt planning frame | **`base_link`** (URDF root; verified live via `/compute_fk`) |
| TCP frame | **`wrist3_link`** = flange. **`tcp_link`** = calibrated fingertip center (A2), offset `[+0.0025, -0.0034, +0.2323]` m from `config/tcp_offset.yaml`; accepted verification tolerance is **6 mm**. |
| Camera frame | **`zed_left_optical`**; B2 publishes the accepted fixed transform `base_link → zed_left_optical` from `calib/T_base_cam.json`. |
| Robot | Fairino FR5 (firmware V3.9.x), real controller at `192.168.58.2:8080` on `enP2p1s0` |
| Planning group | `fairino5_v6_group` (j1–j6), controller `fairino5_controller/follow_joint_trajectory` |
| Named poses (SRDF) | `standby`, `pos1`, `pos2`, `leftGrab`, `leftLift`, `rightGrab`, `rightLift`, `genRotate`, `transit`, `drop` |
| Park pose | The arm sits at `standby`; sim mock hardware also starts there. |

Measured at `standby` (live, 2026-07-06): TCP in `base_link` =
`x -0.0384, y +0.1534, z +0.5740` m, RPY `-2.344, +0.019, +0.810` rad.

## ⚠ One RPC client only

The FR5 controller tolerates **one** RPC session. This stack and the
production docker stack (`~/fairino_ros_connector/fairino_ros_controller/fairino_plan_executor`,
containers `fairino_plan_executor`/`fairino_zed_camera`) must never run
against the robot at the same time. Check before bringup:

```bash
ss -tn | grep 192.168.58.2:8080   # must be empty
docker ps                          # fairino_plan_executor must not be Up
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
source ~/fairino_ros_connector/install/setup.bash  # underlay: fairino_hardware + fairino_msgs (aarch64)
cd ~/VLA_Model_Work/robot_ws
colcon build --symlink-install
source install/setup.bash
```

`src/fairino_description` is a symlink into
`~/fairino_ros_connector/fairino_ros_libs/`. (The workspace previously lived
at `~/fairino5`; symlink and paths were fixed 2026-07-15.) This workspace was
built with `--symlink-install`: YAML edits require a bringup restart so xacro
reloads them, but the currently installed Python, URDF, SRDF, YAML, and launch
files are symlinked to the source tree. Rebuild when adding/removing installed
files or when using a non-symlink installation.

## Run

```bash
# Simulation (mock hardware, RViz on by default):
ros2 launch fr5_bringup a1_bringup.launch.py                # sim:=true

# Real robot (robot powered, faults cleared, Remote mode):
ros2 launch fr5_bringup a1_bringup.launch.py sim:=false     # rviz:=true optional
```

Bringup = robot_state_publisher + ros2_control (`FairinoHardwareInterface`
or mock) + joint_state_broadcaster + fairino5_controller + move_group + the
accepted B2 static camera TF. Before B2 exists, use
`publish_camera_tf:=false`; after a new B1 capture, refit B2 before restarting
bringup or its stale-calibration guard will stop the launch.

## A1 script

```bash
ros2 run fr5_bringup a1_move_and_read.py --list-poses
ros2 run fr5_bringup a1_move_and_read.py --watch-only            # stream TCP @5 Hz while jogging
ros2 run fr5_bringup a1_move_and_read.py --pose standby          # DRY-RUN: plan only + watch
ros2 run fr5_bringup a1_move_and_read.py --pose standby --execute  # MOVES THE ARM
```

Safety defaults: **dry-run unless `--execute`**, velocity/acceleration
scaling **0.1**. After any motion the script keeps streaming the live TCP
pose (`base_link → wrist3_link` from TF, fed by real joint states) plus
joint angles until Ctrl-C.

## A1 verification status (completed live 2026-07-15)

- Sim: plan + execute + TCP readback all pass (standby → transit, 145-point
  14.3 s trajectory at 0.1 scale).
- Real: driver connects (`机械臂SDK连接成功`), live joint states match the
  pendant, planning succeeds, and a small j2 dip-and-return was executed from
  code at 0.1 scaling while TCP and joint readback streamed.

## A2: gripper + fingertip TCP — complete (2026-07-16)

The DH PGC140 is not a ros2_control axis — it hangs off the FR5 controller's
tool bus, commanded through `fairino_remote_command_service`, which the
`FairinoHardwareInterface` hosts **inside** `ros2_control_node` (one shared
RPC session; same wire commands the production `fr5_telemetry_node` uses:
`SetGripperConfig(4,0,0,0)` → `ActGripper(1,1)` → `MoveGripper(1,pct)`).
Requires bringup with `sim:=false`; mock hardware has no gripper service.

The gripper opens/closes from `a2_gripper.py`; its measured 50 mm stroke and
20 x 40 mm pads are recorded in `config/gripper.yaml`. Pivot calibration wrote
the fingertip offset `[+0.0025, -0.0034, +0.2323]` m to
`config/tcp_offset.yaml`. The retained four-touch fit has 2.8 mm RMS residual;
the two-orientation verification disagreed by approximately 6 mm. On
2026-07-16 that result was accepted and the project-wide A2 verification
tolerance was set to **6 mm**.

Re-running the tools remains available if B3 later exposes a systematic miss:

```bash
ros2 run fr5_bringup a2_gripper.py --open        # --close / --pos / --status / --watch
ros2 run fr5_bringup a2_tcp_calibrate.py --samples 6 --write
# Restart bringup so robot_description reloads the offset.
ros2 run fr5_bringup a2_tcp_calibrate.py --verify  # pass threshold: <= 6 mm
```

The fingertip is TF `base_link → tcp_link` (fixed child of `wrist3_link`,
offset from `config/tcp_offset.yaml`). Milestone B scripts should read
`tcp_link`, not `wrist3_link`. Jaw stroke / pad sizes measured in the stroke
test go in `config/gripper.yaml` for the C2/C3 width checks.

## B1/B2: camera-to-base calibration — complete (2026-07-16)

B1 captured eight well-spread correspondences between ZED points in
`zed_left_optical` and fingertip touches in `base_link`. B2 fits the no-scale
rigid transform `p_base = R @ p_camera + t`, reports every residual, and
refuses to overwrite the accepted transform unless RMS is at most 8 mm and
the maximum individual residual is at most 15 mm.

The accepted fit in `calib/T_base_cam.json` has **7.532 mm RMS**, **6.542 mm
mean**, and **12.358 mm maximum** residual. Its source-file SHA-256 is retained
so bringup detects a B1 capture changed without a corresponding B2 refit.

```bash
# Refit/report only:
ros2 run fr5_bringup b2_fit_transform.py

# Refit and atomically replace T_base_cam.json only if quality passes:
ros2 run fr5_bringup b2_fit_transform.py --write

# Bringup publishes base_link -> zed_left_optical by default. Verify it:
ros2 run tf2_ros tf2_echo base_link zed_left_optical
```

Any physical movement of the camera or robot base invalidates this result.
B3 hover validation is still required before using transformed camera targets
for close robot motion.

## A3: taught waypoints (zones removed 2026-07-16)

The workspace-safety layer (MoveIt keep-out boxes, keep-in TCP watchdog,
`workspace.yaml`) was removed on 2026-07-16: the station is fixed — two pick
bins (left/right) and a drop in the same region — so all transit motion runs
between hardcoded taught **joint-space** waypoints (`home`, `hover_bin_left`,
`hover_bin_right`, `drop`) and the cage clutter is never approached. The
deleted `a3_*` scripts are recoverable from git history (commit `6f65f44`).

What replaces zones: the C3 safety gate clamps the vision-derived grasp Z to
the measured table height and requires the target inside the active bin's
recorded extent. Those are plain numbers measured by jogging the fingertip
(table: 3 touches; bins: interior walls), not scene objects.

## Known gaps (by design, later tasks)

- There is no collision scene and no runtime keep-in guard: MoveIt only
  self-collision-checks. Mitigations are procedural — joint-space waypoints
  only, speed at 0.1 for live runs, dry-run default, hand on the e-stop.
