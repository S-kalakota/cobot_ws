# robot_ws — FR5 + MoveIt host workspace (Second_plan.md, Milestone A)

Host-side (Jetson Thor, ROS 2 Jazzy) bringup of the Fairino FR5 driver +
MoveIt, built for the VLA pick pipeline. Created for **Task A1**.

## Facts recorded by A1 (needed by Milestone B)

| Question | Answer |
|---|---|
| MoveIt planning frame | **`base_link`** (URDF root; verified live via `/compute_fk`) |
| TCP frame | **`wrist3_link`** = flange. **`tcp_link`** = fingertip (A2): fixed offset from `config/tcp_offset.yaml` — placeholder until `a2_tcp_calibrate.py --write` solves it. |
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
at `~/fairino5`; symlink and paths fixed 2026-07-15 — **rebuild before the
next launch**, also to pick up the new `tcp_link` / A2 scripts.)

## Run

```bash
# Simulation (mock hardware, RViz on by default):
ros2 launch fr5_bringup a1_bringup.launch.py                # sim:=true

# Real robot (robot powered, faults cleared, Remote mode):
ros2 launch fr5_bringup a1_bringup.launch.py sim:=false     # rviz:=true optional
```

Bringup = robot_state_publisher + ros2_control (`FairinoHardwareInterface`
or mock) + joint_state_broadcaster + fairino5_controller + move_group.

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

## A1 verification status (2026-07-06)

- Sim: plan + execute + TCP readback all pass (standby → transit, 145-point
  14.3 s trajectory at 0.1 scale).
- Real: driver connects (`机械臂SDK连接成功`), live joint states stream and
  match the pendant, dry-run plan to `transit` succeeds. **Live `--execute`
  deliberately not run autonomously** — run it yourself with a hand on the
  e-stop:

  ```bash
  ros2 run fr5_bringup a1_move_and_read.py --pose standby --execute
  ```

  (Arm is already at standby, so the first live run is a near-zero-length
  motion — the safest possible smoke test. Then try `--pose transit --execute`.)

## A2: gripper + fingertip TCP (code written 2026-07-15, not yet run)

The DH PGC140 is not a ros2_control axis — it hangs off the FR5 controller's
tool bus, commanded through `fairino_remote_command_service`, which the
`FairinoHardwareInterface` hosts **inside** `ros2_control_node` (one shared
RPC session; same wire commands the production `fr5_telemetry_node` uses:
`SetGripperConfig(4,0,0,0)` → `ActGripper(1,1)` → `MoveGripper(1,pct)`).
Requires bringup with `sim:=false`; mock hardware has no gripper service.

```bash
# 1. gripper I/O + physical numbers (calipers ready):
ros2 run fr5_bringup a2_gripper.py --activate
ros2 run fr5_bringup a2_gripper.py --open        # --close / --pos 40 / --status / --watch
ros2 run fr5_bringup a2_gripper.py --stroke-test # record results in config/gripper.yaml

# 2. solve the fingertip TCP offset (pivot calibration — touch one fixed
#    point from >= 3 different wrist orientations, jogging via the pendant):
ros2 run fr5_bringup a2_tcp_calibrate.py --samples 4 --write

# 3. rebuild so the URDF picks the offset up, relaunch bringup:
colcon build --symlink-install

# 4. verify (A2 done-when: two-orientation touch test agrees <= 3 mm):
ros2 run fr5_bringup a2_tcp_calibrate.py --verify
```

The fingertip is TF `base_link → tcp_link` (fixed child of `wrist3_link`,
offset from `config/tcp_offset.yaml`). Milestone B scripts should read
`tcp_link`, not `wrist3_link`. Jaw stroke / pad sizes measured in the stroke
test go in `config/gripper.yaml` for the C2/C3 width checks.

## Known gaps (by design, later tasks)

- `tcp_offset.yaml` holds a placeholder (z=0.150) until the calibration above
  has been run on the real arm.
- No table/gantry collision objects in the planning scene yet (Task A3/C4) —
  MoveIt only self-collision-checks; keep speed at 0.1 and watch the arm.
