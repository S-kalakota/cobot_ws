#!/usr/bin/env python3
"""Interactively measure the A3 workspace from TF without commanding motion.

This tool is deliberately read-only with respect to the robot.  The operator
jogs the robot with the Fairino pendant, presses Enter, and the script records
the current ``base_link -> tcp_link`` translation from TF.  It never creates a
motion/action client and never sends a pose, joint, or trajectory command.

The generated workspace file remains disabled by default.  Review its boxes in
RViz before enabling the planning scene, and do not enable the watchdog until
the fingertip TCP has been calibrated and verified.
"""

import argparse
import math
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / 'config' / 'workspace.yaml'
DEFAULT_OBSTACLES = ('camera_gantry', 'monitor')
TCP_UNCERTAINTY_M = 0.006
NOMINAL_FLOOR_CLEARANCE_M = 0.005


class MeasurementCancelled(RuntimeError):
    """Raised when the operator cancels an interactive measurement."""


class TfReader(Node):
    def __init__(self, base_frame, tcp_frame):
        super().__init__('a3_measure_workspace')
        self.base_frame = base_frame
        self.tcp_frame = tcp_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def read_xyz(self, timeout_s=5.0):
        """Return the latest TCP translation as [x, y, z], or None."""
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, self.tcp_frame, rclpy.time.Time())
                t = tf.transform.translation
                return [float(t.x), float(t.y), float(t.z)]
            except Exception:
                continue
        return None


def fmt_point(point):
    return '[' + ', '.join(f'{value:+.4f}' for value in point) + '] m'


def capture_point(node, instruction):
    print(f'\n{instruction}')
    answer = input('Jog manually, let the arm settle, then press Enter '
                   '(or type q to abort): ').strip().lower()
    if answer in {'q', 'quit', 'abort'}:
        raise MeasurementCancelled('operator aborted')
    point = node.read_xyz()
    if point is None:
        raise RuntimeError(
            f'No TF {node.base_frame} -> {node.tcp_frame}. Is bringup running?')
    print(f'Recorded {fmt_point(point)}')
    return point


def capture_points(node, count, label):
    return [capture_point(node, f'{label} {index + 1}/{count}')
            for index in range(count)]


def capture_keep_in_samples(node):
    print('\nKEEP-IN ENVELOPE SAMPLES')
    print('In simulation, visit every taught/named pose and other intended safe '
          'extremes. Capture each one. These samples form an axis-aligned box.')
    samples = []
    while True:
        answer = input(
            f'Position at safe sample {len(samples) + 1}; press Enter to record, '
            "or type 'done' when finished: ").strip().lower()
        if answer in {'done', 'd'}:
            if len(samples) < 3:
                print('Capture at least three varied positions before finishing.')
                continue
            break
        if answer in {'q', 'quit', 'abort'}:
            raise MeasurementCancelled('operator aborted')
        point = node.read_xyz()
        if point is None:
            raise RuntimeError(
                f'No TF {node.base_frame} -> {node.tcp_frame}. Is bringup running?')
        samples.append(point)
        print(f'Recorded {fmt_point(point)}')
    return samples


def ask_float(prompt, default, minimum=0.0):
    while True:
        answer = input(f'{prompt} [{default:.3f} m]: ').strip()
        if not answer:
            return default
        try:
            value = float(answer)
        except ValueError:
            print('Enter a number in metres.')
            continue
        if value <= minimum:
            print(f'Value must be greater than {minimum:.3f} m.')
            continue
        return value


def axis_bounds(points):
    return ([min(point[axis] for point in points) for axis in range(3)],
            [max(point[axis] for point in points) for axis in range(3)])


def make_box(box_id, point_a, point_b, padding, color):
    low = [min(a, b) - padding for a, b in zip(point_a, point_b)]
    high = [max(a, b) + padding for a, b in zip(point_a, point_b)]
    size = [high[index] - low[index] for index in range(3)]
    if any(value <= 0.0 for value in size):
        raise ValueError(f'{box_id} has a non-positive box dimension')
    return {
        'id': box_id,
        'enabled': True,
        'center': [(low[index] + high[index]) / 2.0 for index in range(3)],
        'size': size,
        'color': color,
    }


def rounded(values):
    return [round(float(value), 6) for value in values]


def build_workspace(node, args):
    print('\nA3 WORKSPACE MEASUREMENT')
    print('This program NEVER moves the robot. It only reads TF after you jog '
          'the robot manually and press Enter.')
    print(f'Recording {args.frame} -> {args.tcp_frame}.')
    if not args.tcp_calibrated:
        print('WARNING: TCP is marked provisional. Measurements may be useful '
              'for simulation, but must be repeated after A2 calibration.')

    table_points = capture_points(
        node, 3,
        'Touch the SAME TCP point gently to a spread-out spot on the table surface.')
    table_z_values = [point[2] for point in table_points]
    table_plane_z = sum(table_z_values) / len(table_z_values)
    table_spread = max(table_z_values) - min(table_z_values)
    print(f'Table z average: {table_plane_z:+.4f} m; '
          f'spread: {table_spread * 1000.0:.1f} mm')
    if table_spread > 0.008:
        print('WARNING: table touches differ by more than 8 mm. Recheck the TCP '
              'point, wrist orientation, and table contact.')

    table_corners = capture_points(
        node, 4,
        'Touch or hover at a different table top corner/edge limit.')
    table_low, table_high = axis_bounds(table_corners)
    table_thickness = ask_float('Table collision-box thickness', 0.050)
    table_pad_xy = args.table_padding
    table_collision_top_z = table_plane_z + TCP_UNCERTAINTY_M
    table_box = {
        'id': 'table',
        'enabled': True,
        'center': [
            (table_low[0] + table_high[0]) / 2.0,
            (table_low[1] + table_high[1]) / 2.0,
            table_collision_top_z - table_thickness / 2.0,
        ],
        'size': [
            table_high[0] - table_low[0] + 2.0 * table_pad_xy,
            table_high[1] - table_low[1] + 2.0 * table_pad_xy,
            table_thickness,
        ],
        'color': [0.45, 0.45, 0.45, 1.0],
    }

    colors = ([0.95, 0.55, 0.10, 1.0], [0.85, 0.20, 0.20, 1.0])
    boxes = [table_box]
    skipped = []
    for index, name in enumerate(args.obstacle):
        answer = input(
            f'\nMeasure keep-out box {name!r}? [y/N]: ').strip().lower()
        if answer not in {'y', 'yes'}:
            skipped.append(name)
            continue
        print('Capture two opposite AABB corners that enclose the obstacle. '
              'Hover—do not press the robot into the obstacle.')
        point_a = capture_point(node, f'{name}: first bounding corner')
        point_b = capture_point(node, f'{name}: opposite bounding corner')
        color = colors[index % len(colors)]
        boxes.append(make_box(name, point_a, point_b,
                              args.obstacle_padding, color))

    keep_in_samples = capture_keep_in_samples(node)
    keep_low, keep_high = axis_bounds(keep_in_samples)
    keep_low = [value - args.keep_in_margin for value in keep_low]
    keep_high = [value + args.keep_in_margin for value in keep_high]
    extents = [keep_high[i] - keep_low[i] for i in range(3)]
    if any(extent < 0.050 for extent in extents):
        print('WARNING: at least one keep-in axis spans less than 50 mm. Add '
              'more varied samples before enabling the watchdog.')

    measured_by = args.measured_by or os.environ.get('USER') or 'unknown'
    complete = not skipped and table_spread <= 0.008
    notes = []
    if skipped:
        notes.append('Unmeasured obstacles: ' + ', '.join(skipped))
    if not args.tcp_calibrated:
        notes.append('TCP provisional; remeasure after A2 calibration')
    if table_spread > 0.008:
        notes.append('Table touch spread exceeds 8 mm')

    return {
        'schema_version': 1,
        'frame_id': args.frame,
        'tcp_frame': args.tcp_frame,
        'metadata': {
            'measured_at': datetime.now(timezone.utc).isoformat(),
            'measured_by': measured_by,
            'tcp_calibrated': bool(args.tcp_calibrated),
            'measurement_complete': complete,
            'notes': '; '.join(notes) if notes else 'Review in RViz before enabling.',
        },
        'measurements': {
            'table_surface_points': [rounded(point) for point in table_points],
            'table_plane_z': round(table_plane_z, 6),
            'table_collision_top_z': round(table_collision_top_z, 6),
            'keep_in_samples': [rounded(point) for point in keep_in_samples],
        },
        'planning_scene': {
            # Deliberately false. Review boxes in the YAML first, then enable.
            'enabled': False,
            'boxes': [
                {**box, 'center': rounded(box['center']),
                 'size': rounded(box['size'])}
                for box in boxes
            ],
        },
        'keep_in': {
            # Deliberately false until the bounds pass simulation testing.
            'enabled': False,
            'min': rounded(keep_low),
            'max': rounded(keep_high),
        },
        'watchdog': {
            # Deliberately false. Never enable with a provisional TCP.
            'enabled': False,
            'rate_hz': 20.0,
            'startup_grace_s': 5.0,
            'tf_stale_timeout_s': 0.5,
            # Keep the physical TCP at least 5 mm above the measured table
            # even at the accepted 6 mm A2 pose uncertainty.
            'tcp_uncertainty_m': TCP_UNCERTAINTY_M,
            'nominal_floor_clearance_m': NOMINAL_FLOOR_CLEARANCE_M,
            'floor_clearance_m': (
                TCP_UNCERTAINTY_M + NOMINAL_FLOOR_CLEARANCE_M),
            'boundary_tolerance_m': 0.0,
            'cancel_repeat_s': 0.10,
            'cancel_services': [
                '/move_action/_action/cancel_goal',
                '/execute_trajectory/_action/cancel_goal',
                '/fairino5_controller/follow_joint_trajectory/_action/cancel_goal',
            ],
        },
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--frame', default='base_link')
    parser.add_argument('--tcp-frame', default='tcp_link')
    parser.add_argument('--obstacle', action='append', default=None,
                        help='keep-out obstacle name; repeat for multiple boxes')
    parser.add_argument('--obstacle-padding', type=float, default=0.050,
                        help='padding added on every obstacle side (default 0.05 m)')
    parser.add_argument('--table-padding', type=float, default=0.020,
                        help='horizontal table padding (default 0.02 m)')
    parser.add_argument('--keep-in-margin', type=float, default=0.100,
                        help='margin around captured safe poses (default 0.10 m)')
    parser.add_argument('--tcp-calibrated', action='store_true',
                        help='record that A2 TCP verification has passed')
    parser.add_argument('--measured-by')
    args = parser.parse_args(argv)
    args.obstacle = args.obstacle or list(DEFAULT_OBSTACLES)
    for name, value in (
            ('obstacle padding', args.obstacle_padding),
            ('table padding', args.table_padding),
            ('keep-in margin', args.keep_in_margin)):
        if not math.isfinite(value) or value < 0.0:
            parser.error(f'{name} must be a finite, non-negative number')
    return args


def main(argv=None):
    args = parse_args(argv)
    rclpy.init()
    node = TfReader(args.frame, args.tcp_frame)
    try:
        workspace = build_workspace(node, args)
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(yaml.safe_dump(workspace, sort_keys=False), encoding='utf-8')
        print(f'\nWrote {output}')
        print('Both planning_scene.enabled and watchdog.enabled remain false. '
              'Review the values and test in simulation before enabling them.')
        return 0
    except MeasurementCancelled as exc:
        print(f'Cancelled: {exc}', file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
