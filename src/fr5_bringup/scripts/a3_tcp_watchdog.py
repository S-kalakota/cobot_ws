#!/usr/bin/env python3
"""Latched A3 TCP keep-in watchdog.

At the configured rate this node checks ``base_link -> tcp_link`` against the
keep-in AABB and table floor in ``workspace.yaml``.  On a breach it sends
cancel-all requests to configured ROS action cancel services and remains
latched until an operator calls ``~/acknowledge``.  It never sends a motion
goal and never performs an automatic recovery move.

This is a software protection layer, not a safety-rated stop.  It does not
replace the Fairino e-stop, protective stop, speed limits, or human oversight.
"""

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


ARMED = 'ARMED'
LATCHED = 'LATCHED'
RECOVERY = 'RECOVERY'
DISABLED = 'DISABLED'


def default_workspace_path():
    try:
        return Path(get_package_share_directory('fr5_bringup')) / 'config' / 'workspace.yaml'
    except Exception:
        return Path(__file__).resolve().parents[1] / 'config' / 'workspace.yaml'


def finite_vector(value, length, label):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f'{label} must be a {length}-element list')
    result = []
    for item in value:
        if not isinstance(item, (int, float)) or not math.isfinite(float(item)):
            raise ValueError(f'{label} contains a non-finite/non-numeric value')
        result.append(float(item))
    return result


def finite_number(value, label, minimum=None, strictly_positive=False):
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f'{label} must be finite and numeric')
    result = float(value)
    if strictly_positive and result <= 0.0:
        raise ValueError(f'{label} must be positive')
    if minimum is not None and result < minimum:
        raise ValueError(f'{label} must be >= {minimum}')
    return result


def load_workspace(path):
    path = Path(path).expanduser().resolve()
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
    except OSError as exc:
        raise ValueError(f'cannot read {path}: {exc}') from exc
    except yaml.YAMLError as exc:
        raise ValueError(f'invalid YAML in {path}: {exc}') from exc
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('workspace must be a schema_version 1 mapping')
    for key in ('frame_id', 'tcp_frame'):
        if not isinstance(data.get(key), str) or not data[key]:
            raise ValueError(f'{key} must be a non-empty string')
    keep_in = data.get('keep_in')
    watchdog = data.get('watchdog')
    if not isinstance(keep_in, dict) or not isinstance(keep_in.get('enabled'), bool):
        raise ValueError('keep_in.enabled must be true or false')
    if not isinstance(watchdog, dict) or not isinstance(watchdog.get('enabled'), bool):
        raise ValueError('watchdog.enabled must be true or false')

    if keep_in['enabled']:
        low = finite_vector(keep_in.get('min'), 3, 'keep_in.min')
        high = finite_vector(keep_in.get('max'), 3, 'keep_in.max')
        if any(low[index] >= high[index] for index in range(3)):
            raise ValueError('every keep_in.min value must be below keep_in.max')
    if watchdog['enabled']:
        if not keep_in['enabled']:
            raise ValueError('watchdog is enabled while keep_in is disabled')
        if not data.get('metadata', {}).get('tcp_calibrated', False):
            raise ValueError('watchdog cannot be enabled with an uncalibrated TCP')
        table_z = data.get('measurements', {}).get('table_plane_z')
        finite_number(table_z, 'measurements.table_plane_z')
        finite_number(watchdog.get('rate_hz'), 'watchdog.rate_hz',
                      strictly_positive=True)
        finite_number(watchdog.get('startup_grace_s'),
                      'watchdog.startup_grace_s', minimum=0.0)
        finite_number(watchdog.get('tf_stale_timeout_s'),
                      'watchdog.tf_stale_timeout_s', strictly_positive=True)
        tcp_uncertainty = finite_number(
            watchdog.get('tcp_uncertainty_m'),
            'watchdog.tcp_uncertainty_m', minimum=0.0)
        nominal_floor_clearance = finite_number(
            watchdog.get('nominal_floor_clearance_m'),
            'watchdog.nominal_floor_clearance_m', minimum=0.0)
        floor_clearance = finite_number(
            watchdog.get('floor_clearance_m'),
            'watchdog.floor_clearance_m', minimum=0.0)
        required_floor_clearance = tcp_uncertainty + nominal_floor_clearance
        if floor_clearance + 1e-12 < required_floor_clearance:
            raise ValueError(
                'watchdog.floor_clearance_m must be at least '
                'tcp_uncertainty_m + nominal_floor_clearance_m '
                f'({required_floor_clearance:.4f} m)')
        if any(low[index] + tcp_uncertainty >= high[index] - tcp_uncertainty
               for index in range(3)):
            raise ValueError(
                'keep_in bounds must remain non-empty after applying the TCP '
                'uncertainty inward on every side')
        finite_number(watchdog.get('boundary_tolerance_m'),
                      'watchdog.boundary_tolerance_m', minimum=0.0)
        finite_number(watchdog.get('cancel_repeat_s'),
                      'watchdog.cancel_repeat_s', strictly_positive=True)
        services = watchdog.get('cancel_services')
        if not isinstance(services, list) or not services or not all(
                isinstance(name, str) and name.startswith('/') for name in services):
            raise ValueError('watchdog.cancel_services must contain absolute service names')
    return path, data


class TcpWatchdog(Node):
    def __init__(self, workspace_override=None):
        super().__init__('a3_tcp_watchdog')
        self.declare_parameter('workspace', str(default_workspace_path()))
        workspace_path = workspace_override or self.get_parameter('workspace').value
        self.workspace_path, self.workspace = load_workspace(workspace_path)
        self.config = self.workspace['watchdog']
        self.enabled = self.config['enabled']
        self.frame_id = self.workspace['frame_id']
        self.tcp_frame = self.workspace['tcp_frame']
        self.state = DISABLED if not self.enabled else ARMED
        self.reason = 'watchdog disabled' if not self.enabled else 'within bounds'
        self.position = None
        self.started_at = time.monotonic()
        self.last_tf_at = None
        self.last_cancel_at = 0.0
        self.cancel_pending = set()

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(String, '~/state', qos)
        self.create_service(Trigger, '~/acknowledge', self._acknowledge)
        self.create_service(Trigger, '~/status', self._status)
        self._publish_state()

        self.cancel_clients = {}
        if not self.enabled:
            self.get_logger().warn(
                f'TCP watchdog is DISABLED in {self.workspace_path}; no goals '
                'will be cancelled.')
            return

        self.low = finite_vector(self.workspace['keep_in']['min'], 3, 'keep_in.min')
        self.high = finite_vector(self.workspace['keep_in']['max'], 3, 'keep_in.max')
        table_z = finite_number(
            self.workspace['measurements']['table_plane_z'], 'table_plane_z')
        self.tcp_uncertainty = finite_number(
            self.config['tcp_uncertainty_m'], 'tcp_uncertainty_m', minimum=0.0)
        self.nominal_floor_clearance = finite_number(
            self.config['nominal_floor_clearance_m'],
            'nominal_floor_clearance_m', minimum=0.0)
        self.floor_clearance = finite_number(
            self.config['floor_clearance_m'], 'floor_clearance_m', minimum=0.0)
        self.floor_z = table_z + self.floor_clearance
        self.effective_low = [
            value + self.tcp_uncertainty for value in self.low]
        self.effective_high = [
            value - self.tcp_uncertainty for value in self.high]
        self.tolerance = finite_number(
            self.config['boundary_tolerance_m'], 'boundary_tolerance_m', minimum=0.0)
        self.startup_grace = finite_number(
            self.config['startup_grace_s'], 'startup_grace_s', minimum=0.0)
        self.tf_stale_timeout = finite_number(
            self.config['tf_stale_timeout_s'], 'tf_stale_timeout_s',
            strictly_positive=True)
        self.cancel_repeat = finite_number(
            self.config['cancel_repeat_s'], 'cancel_repeat_s', strictly_positive=True)
        for service_name in self.config['cancel_services']:
            self.cancel_clients[service_name] = self.create_client(
                CancelGoal, service_name)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        rate_hz = finite_number(self.config['rate_hz'], 'rate_hz',
                                strictly_positive=True)
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'ARMED at {rate_hz:.1f} Hz: {self.frame_id} -> {self.tcp_frame}, '
            f'effective keep-in min={self.effective_low}, '
            f'max={self.effective_high}, floor_z={self.floor_z:.4f} '
            f'(clearance={self.floor_clearance:.3f} m = '
            f'{self.tcp_uncertainty:.3f} m TCP uncertainty + '
            f'{self.nominal_floor_clearance:.3f} m nominal)')

    def _read_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id, self.tcp_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return [float(t.x), float(t.y), float(t.z)]

    def _breach_reasons(self, position):
        axis_names = ('x', 'y', 'z')
        reasons = []
        for index, axis in enumerate(axis_names):
            if position[index] < self.effective_low[index] - self.tolerance:
                reasons.append(
                    f'{axis}={position[index]:.4f} below '
                    f'{self.effective_low[index]:.4f}')
            if position[index] > self.effective_high[index] + self.tolerance:
                reasons.append(
                    f'{axis}={position[index]:.4f} above '
                    f'{self.effective_high[index]:.4f}')
        if position[2] < self.floor_z - self.tolerance:
            reasons.append(
                f'z={position[2]:.4f} below table floor {self.floor_z:.4f}')
        return reasons

    def _tick(self):
        now = time.monotonic()
        position = self._read_position()
        if position is not None:
            self.position = position
            self.last_tf_at = now
            reasons = self._breach_reasons(position)
            if self.state == RECOVERY:
                if not reasons:
                    self.state = ARMED
                    self.reason = 're-entered bounds; watchdog re-armed'
                    self.get_logger().info(self.reason)
                    self._publish_state()
                return
            if self.state == ARMED and reasons:
                self._latch('; '.join(reasons))
        else:
            age = now - (self.last_tf_at or self.started_at)
            if now - self.started_at > self.startup_grace and age > self.tf_stale_timeout:
                if self.state in {ARMED, RECOVERY}:
                    self._latch(f'TF unavailable/stale for {age:.2f} s')

        if self.state == LATCHED and now - self.last_cancel_at >= self.cancel_repeat:
            self._cancel_all_goals()

    def _latch(self, reason):
        self.state = LATCHED
        self.reason = reason
        self.get_logger().error(
            f'WORKSPACE BREACH: {reason}. Cancelling ROS motion goals and '
            'remaining latched until ~/acknowledge is called.')
        self._publish_state()
        self._cancel_all_goals()

    def _cancel_all_goals(self):
        self.last_cancel_at = time.monotonic()
        for name, client in self.cancel_clients.items():
            if name in self.cancel_pending:
                continue
            if not client.service_is_ready():
                self.get_logger().warn(
                    f'Cancel service unavailable: {name}',
                    throttle_duration_sec=5.0)
                continue
            request = CancelGoal.Request()
            # A zero UUID and zero timestamp means cancel every goal.
            request.goal_info.goal_id.uuid = [0] * 16
            self.cancel_pending.add(name)
            future = client.call_async(request)
            future.add_done_callback(
                lambda completed, service=name: self._cancel_done(service, completed))

    def _cancel_done(self, service, future):
        self.cancel_pending.discard(service)
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Cancel request failed for {service}: {exc}')
            return
        if response.return_code == CancelGoal.Response.ERROR_NONE:
            self.get_logger().warn(
                f'Cancel accepted by {service}: '
                f'{len(response.goals_canceling)} goal(s) cancelling')

    def _acknowledge(self, _request, response):
        if not self.enabled:
            response.success = False
            response.message = 'watchdog is disabled'
            return response
        if self.state != LATCHED:
            response.success = False
            response.message = f'watchdog is {self.state}, not latched'
            return response
        now = time.monotonic()
        if self.last_tf_at is None or now - self.last_tf_at > self.tf_stale_timeout:
            response.success = False
            response.message = 'cannot acknowledge without fresh TCP TF'
            return response
        reasons = self._breach_reasons(self.position)
        if reasons:
            self.state = RECOVERY
            self.reason = ('operator acknowledged; recovery motion may be commanded '
                           'until the TCP re-enters bounds')
        else:
            self.state = ARMED
            self.reason = 'operator acknowledged; watchdog re-armed'
        self.get_logger().warn(self.reason)
        self._publish_state()
        response.success = True
        response.message = self.reason
        return response

    def _status(self, _request, response):
        response.success = self.enabled and self.state == ARMED
        response.message = self._state_text()
        return response

    def _state_text(self):
        position = ('unknown' if self.position is None else
                    '[' + ', '.join(f'{value:+.4f}' for value in self.position) + ']')
        return f'state={self.state}; position={position}; reason={self.reason}'

    def _publish_state(self):
        message = String()
        message.data = self._state_text()
        self.state_pub.publish(message)


def parse_cli(raw_args):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path)
    parser.add_argument('--validate-only', action='store_true')
    return parser.parse_args(remove_ros_args(args=raw_args)[1:])


def main(argv=None):
    raw_args = list(sys.argv if argv is None else [sys.argv[0], *argv])
    args = parse_cli(raw_args)
    workspace = args.workspace or default_workspace_path()
    if args.validate_only:
        try:
            path, data = load_workspace(workspace)
            print(f'valid workspace: {path} '
                  f'(watchdog_enabled={data["watchdog"]["enabled"]}, '
                  f'keep_in_enabled={data["keep_in"]["enabled"]})')
            return 0
        except ValueError as exc:
            print(f'INVALID: {exc}', file=sys.stderr)
            return 1

    rclpy.init(args=raw_args)
    node = None
    try:
        node = TcpWatchdog(args.workspace)
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except ValueError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
