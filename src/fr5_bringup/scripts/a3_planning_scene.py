#!/usr/bin/env python3
"""Publish A3 keep-out boxes into the MoveIt planning scene.

The node reads ``workspace.yaml`` and calls MoveIt's ``apply_planning_scene``
service.  It never requests or executes robot motion.  A disabled or invalid
configuration fails closed: no collision objects are published.
"""

import argparse
import math
import sys
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger


def default_workspace_path():
    try:
        return Path(get_package_share_directory('fr5_bringup')) / 'config' / 'workspace.yaml'
    except Exception:
        return Path(__file__).resolve().parents[1] / 'config' / 'workspace.yaml'


def finite_vector(value, length, label, positive=False):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f'{label} must be a {length}-element list')
    result = []
    for item in value:
        if not isinstance(item, (int, float)) or not math.isfinite(float(item)):
            raise ValueError(f'{label} contains a non-finite/non-numeric value')
        number = float(item)
        if positive and number <= 0.0:
            raise ValueError(f'{label} values must be positive')
        result.append(number)
    return result


def load_workspace(path):
    path = Path(path).expanduser().resolve()
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
    except OSError as exc:
        raise ValueError(f'cannot read {path}: {exc}') from exc
    except yaml.YAMLError as exc:
        raise ValueError(f'invalid YAML in {path}: {exc}') from exc
    if not isinstance(data, dict):
        raise ValueError('workspace root must be a mapping')
    if data.get('schema_version') != 1:
        raise ValueError('workspace schema_version must be 1')
    frame_id = data.get('frame_id')
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError('frame_id must be a non-empty string')
    scene = data.get('planning_scene')
    if not isinstance(scene, dict) or not isinstance(scene.get('enabled'), bool):
        raise ValueError('planning_scene.enabled must be true or false')
    boxes = scene.get('boxes', [])
    if not isinstance(boxes, list):
        raise ValueError('planning_scene.boxes must be a list')
    seen = set()
    for index, box in enumerate(boxes):
        label = f'planning_scene.boxes[{index}]'
        if not isinstance(box, dict):
            raise ValueError(f'{label} must be a mapping')
        box_id = box.get('id')
        if not isinstance(box_id, str) or not box_id:
            raise ValueError(f'{label}.id must be a non-empty string')
        if box_id in seen:
            raise ValueError(f'duplicate collision-object id {box_id!r}')
        seen.add(box_id)
        if not isinstance(box.get('enabled'), bool):
            raise ValueError(f'{label}.enabled must be true or false')
        # Disabled placeholders are allowed to retain null geometry.
        if box['enabled']:
            finite_vector(box.get('center'), 3, f'{label}.center')
            finite_vector(box.get('size'), 3, f'{label}.size', positive=True)
            finite_vector(box.get('color', [0.7, 0.3, 0.2, 1.0]), 4,
                          f'{label}.color')
    if scene['enabled'] and not any(box.get('enabled') for box in boxes):
        raise ValueError('planning scene is enabled but has no enabled boxes')
    return path, data


def make_collision_object(frame_id, box):
    center = finite_vector(box['center'], 3, f"{box['id']}.center")
    size = finite_vector(box['size'], 3, f"{box['id']}.size", positive=True)
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = size
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = center
    pose.orientation.w = 1.0
    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = box['id']
    obj.primitives = [primitive]
    obj.primitive_poses = [pose]
    obj.operation = CollisionObject.ADD
    return obj


def make_planning_scene(workspace):
    scene_msg = PlanningScene()
    scene_msg.is_diff = True
    for box in workspace['planning_scene']['boxes']:
        if not box.get('enabled'):
            continue
        scene_msg.world.collision_objects.append(
            make_collision_object(workspace['frame_id'], box))
        rgba = finite_vector(box.get('color', [0.7, 0.3, 0.2, 1.0]), 4,
                             f"{box['id']}.color")
        color = ObjectColor()
        color.id = box['id']
        color.color.r, color.color.g, color.color.b, color.color.a = rgba
        scene_msg.object_colors.append(color)
    return scene_msg


class PlanningScenePublisher(Node):
    def __init__(self, workspace_override=None):
        super().__init__('a3_planning_scene')
        self.declare_parameter('workspace', str(default_workspace_path()))
        self.declare_parameter('apply_service', '/apply_planning_scene')
        self.declare_parameter('reapply_period_s', 10.0)
        workspace_path = workspace_override or self.get_parameter('workspace').value
        self.workspace_path, self.workspace = load_workspace(workspace_path)
        self.enabled = self.workspace['planning_scene']['enabled']
        self.pending = False
        self.applied_once = False
        self.client = self.create_client(
            ApplyPlanningScene, self.get_parameter('apply_service').value)
        self.create_service(Trigger, '~/reapply', self._on_reapply)

        if not self.enabled:
            self.get_logger().warn(
                f'Planning scene is DISABLED in {self.workspace_path}; no '
                'collision objects will be published.')
            return

        period = float(self.get_parameter('reapply_period_s').value)
        if period <= 0.0:
            raise ValueError('reapply_period_s must be positive')
        self.timer = self.create_timer(min(period, 0.5), self._initial_apply)
        self.reapply_period = period
        self.get_logger().info(
            f'Loaded {len(make_planning_scene(self.workspace).world.collision_objects)} '
            f'collision boxes from {self.workspace_path}')

    def _initial_apply(self):
        if self.applied_once:
            self.timer.cancel()
            self.timer = self.create_timer(self.reapply_period, self.apply_scene)
            return
        self.apply_scene()

    def apply_scene(self):
        if not self.enabled or self.pending:
            return False
        if not self.client.service_is_ready():
            self.get_logger().info(
                'Waiting for MoveIt /apply_planning_scene service...',
                throttle_duration_sec=5.0)
            return False
        request = ApplyPlanningScene.Request()
        request.scene = make_planning_scene(self.workspace)
        self.pending = True
        future = self.client.call_async(request)
        future.add_done_callback(self._on_applied)
        return True

    def _on_applied(self, future):
        self.pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Planning-scene service failed: {exc}')
            return
        if not response.success:
            self.get_logger().error('MoveIt rejected the A3 planning-scene update')
            return
        self.applied_once = True
        self.get_logger().info('A3 keep-out collision boxes applied to MoveIt')

    def _on_reapply(self, _request, response):
        if not self.enabled:
            response.success = False
            response.message = 'planning_scene.enabled is false'
            return response
        response.success = self.apply_scene()
        response.message = ('planning-scene request submitted' if response.success
                            else 'service unavailable or request already pending')
        return response


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
            count = sum(1 for box in data['planning_scene']['boxes']
                        if box.get('enabled'))
            print(f'valid workspace: {path} '
                  f'(scene_enabled={data["planning_scene"]["enabled"]}, '
                  f'enabled_boxes={count})')
            return 0
        except ValueError as exc:
            print(f'INVALID: {exc}', file=sys.stderr)
            return 1

    rclpy.init(args=raw_args)
    node = None
    try:
        node = PlanningScenePublisher(args.workspace)
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
