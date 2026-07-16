# Next time — where we are and what's next

Session notes, 2026-07-15. Continues `Second_plan.md`. All robot code lives in
`~/VLA_Model_Work/robot_ws` (package `fr5_bringup`), pushed to
`github.com/S-kalakota/cobot_ws` (the current canonical remote; the older
`S-kalakota/robot_ws` repository contains the same pre-update tree).

## Completed

### Milestone A1 — command + read the robot ✅ (verified live)
- Bringup: `ros2 launch fr5_bringup a1_bringup.launch.py sim:=false`
- Moved the arm from code (small j2 dip and back at 0.1 speed) and read the
  live TCP round-trip: standby TCP = `x -0.0384, y +0.1534, z +0.5740`.
- Facts recorded: MoveIt plans in `base_link`, group `fairino5_v6_group`,
  robot at `192.168.58.2` (firmware V3.9.x), ROS 2 Jazzy on the Thor.
- Soft stop that works without the e-stop:
  `ros2 topic pub --once /trajectory_execution_event std_msgs/msg/String "{data: stop}"`
  then re-command `--pose standby --execute` (MoveIt replans from wherever
  the arm is).

### Milestone A2 — gripper + fingertip TCP ✅
- Gripper (DH PGC140) works from code: `a2_gripper.py --open/--close/--pos/--stroke-test`.
  Measured (in `config/gripper.yaml`): max jaw stroke **50 mm**, pads touch at
  0%, pads 20×40 mm. Production grasp settings (from the old Lua program):
  close = 71% @ force 57, open = 100% @ force 49 → boxes are ~35 mm across.
- Fingertip TCP calibrated by pivot touches (`a2_tcp_calibrate.py`):
  offset `x 0.0025, y -0.0034, z 0.2323` m in `config/tcp_offset.yaml`,
  4-touch fit, RMS 2.8 mm. TF `base_link → tcp_link` is now the fingertip.
- The two-touch verify landed at ~6 mm. On 2026-07-16 this was accepted as the
  project-wide A2 TCP verification tolerance; A2 is complete. Dependent safety
  margins must include that 6 mm uncertainty. If Milestone B's hover test
  misses by >1 cm, the first diagnostic remains refitting the TCP with
  `a2_tcp_calibrate.py --samples 6 --write`, large wrist tilts (~40°), a
  bringup restart, and `--verify`.

### Milestone B1/B2 — camera-to-base transform ✅ (2026-07-16)
- B1 retained 8 well-spread ZED-point ↔ fingertip-touch pairs in
  `calib/calib_points.json`.
- B2 Kabsch fit passes: **7.532 mm RMS**, **6.542 mm mean**, **12.358 mm max**.
- Accepted transform is `calib/T_base_cam.json`; `b2_fit_transform.py --write`
  refuses failed fits and records per-point residuals plus the source hash.
- `a1_bringup.launch.py` now publishes `base_link → zed_left_optical` by
  default and rejects failed or stale calibration files. Restart bringup to
  load it, then verify with
  `ros2 run tf2_ros tf2_echo base_link zed_left_optical`.
- B2 is numerically complete; **B3 physical hover validation is next** before
  any close approach to a vision target.

### Infrastructure fixed along the way
- `~/fairino5` was renamed to `~/fairino_ros_connector`; re-pointed the
  `robot_ws/src/fairino_description` symlink and fixed broken absolute
  `libfairino.so.2` symlinks in the old install (now relative).
- The host Fairino driver was a stale build with no gripper channel. Replaced
  `~/fairino_ros_connector/install/fairino_hardware/.../libfairino_hardware.so`
  with a fresh build of `fairino_hardware_v3_9_6` (source symlinked into
  `robot_ws/src`, backup kept as `*.stale-20260715`). This driver hosts
  `/fairino_remote_command_service` + `/nonrt_state_data` inside
  ros2_control (one shared RPC session — same as production).
- Updated `leftGrab` / `rightGrab` SRDF poses were applied 2026-07-16 from the
  latest taught start points in `db/plans.sqlite` (`leftgrab_to_leftlift` and
  `rightgrab_to_rightlift`). The SRDF now matches the latest taught
  trajectories.

### Hard-won gotchas (read before debugging)
1. **`tcp_offset.yaml` is baked in at LAUNCH time.** Editing it does nothing
   until the bringup is restarted. Two verify runs failed at 31/42 mm purely
   because the old placeholder (z=0.15) was still loaded. Check what's live:
   the running URDF on `/robot_description` must show the yaml's values.
2. Ctrl-C in the launch terminal kills the whole driver stack — the script
   terminal is where Ctrl-C is safe.
3. Only ONE RPC session to the FR5: stop the docker production stack
   (`fairino_plan_executor`) before `sim:=false`, and vice versa.
4. Every new terminal needs `source ~/VLA_Model_Work/robot_ws/install/setup.bash`.
5. `/home/team/fairino_db` is empty; the real DB is
   `~/fairino_ros_connector/fairino_ros_controller/db/plans.sqlite`
   (11 taught trajectories = the full pick choreography, incl. a fixed drop
   pose — answers Second_plan open question #3).

## Milestone A3 — zones removed (2026-07-16)

The workspace-safety code (`a3_measure_workspace.py`, `a3_planning_scene.py`,
`a3_tcp_watchdog.py`, `config/workspace.yaml`) was **deleted** on 2026-07-16
after the scope change: the station is fixed (two pick bins left/right, drop in
the same region), transit runs only between taught joint-space waypoints, and
the cage clutter is never approached. Recoverable from git history (`6f65f44`)
if the layout changes. A3 is now: teach `home`, `hover_bin_left`,
`hover_bin_right`, `drop` as joint configurations; record table Z (3 touches)
and each bin's interior extent for the C3 gate clamps.

## What's next (in order)

1. **A3 waypoints:** jog to each station, save joint values for `home`,
   `hover_bin_left`, `hover_bin_right`, `drop`; replay all four at 0.1 speed.
   The taught pick choreography in `db/plans.sqlite` (gotcha #5) is a good
   source for the exact bin/drop poses.
2. **A3 numbers for the gate:** 3 table touches → table Z; jog fingertip to
   each bin's interior walls → bin extents in `base_link`. Plain numbers,
   no scene objects.
3. **Milestone B3/B4:** physically validate the accepted B2 transform by
   hovering 100 mm above at least five camera-selected spots (≤15 mm miss),
   then add the AprilTag/ChArUco camera-bump tripwire. B1/B2 are complete.
7. Then Milestone C (table plane, `GraspTarget`, safety gate) per
   `Second_plan.md` step list.

## Also parked
- `mask_service.py` (711-line resident masking daemon, Daemon plan) is
  committed on the `Daemon` branch of SAM_3_implementation — not merged to
  `main` yet.
- The repo's symlinks (`src/fairino_description`, `src/fairino_hardware_v3_9_6`)
  are absolute paths — they dangle on any machine that isn't the Thor.
