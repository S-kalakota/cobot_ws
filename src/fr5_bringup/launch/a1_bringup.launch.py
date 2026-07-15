"""FR5 driver + MoveIt bringup for Task A1 (Second_plan.md, Milestone A).

Starts: robot_state_publisher, ros2_control (mock or fairino_hardware),
joint_state_broadcaster -> fairino5_controller spawner chain, move_group, and
the A3 planning-scene + TCP-watchdog nodes. Optional RViz (rviz:=true;
defaults to the value of `sim`). A3 behavior is controlled by
``config/workspace.yaml`` and ships disabled until measurements are reviewed.

Args:
  sim   -- 'true' (default) => mock_components hardware. 'false' => real FR5
           via fairino_hardware/FairinoHardwareInterface (RPC to 192.168.58.2).
           IMPORTANT: the controller tolerates only one RPC client - make sure
           the fairino_plan_executor docker stack is stopped before sim:=false.
  rviz  -- 'true'/'false'. Defaults to `sim`.

MoveIt plans in frame `base_link` (URDF root); the arm group is
`fairino5_v6_group` with tip link `wrist3_link`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _make_nodes(context, *args, **kwargs):
    sim = context.launch_configurations['sim'].strip().lower()
    use_fake = 'true' if sim == 'true' else 'false'

    moveit_config = (
        MoveItConfigsBuilder('fairino5_v6_robot', package_name='fr5_bringup')
        .robot_description(
            file_path='urdf/fr5.urdf.xacro',
            mappings={'use_fake_hardware': use_fake},
        )
        .robot_description_semantic(file_path='config/fr5.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[moveit_config.robot_description],
    )

    import os
    controllers_yaml = os.path.join(
        moveit_config.package_path, 'config', 'ros2_controllers.yaml')
    cm = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[moveit_config.robot_description, controllers_yaml],
        output='screen',
    )

    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    arm_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['fairino5_controller', '--controller-manager', '/controller_manager'],
    )
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner]))

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    # A3 safety nodes are always present so the safety layer cannot be forgotten.
    # Their checked-in workspace.yaml is disabled, so a fresh checkout publishes
    # no collision geometry and cancels no goals until measured values are reviewed.
    workspace_yaml = os.path.join(
        moveit_config.package_path, 'config', 'workspace.yaml')
    planning_scene = Node(
        package='fr5_bringup', executable='a3_planning_scene.py',
        output='screen', parameters=[{'workspace': workspace_yaml}],
    )
    tcp_watchdog = Node(
        package='fr5_bringup', executable='a3_tcp_watchdog.py',
        output='screen', parameters=[{'workspace': workspace_yaml}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(moveit_config.package_path, 'config', 'moveit.rviz')],
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [rsp, cm, jsb_spawner, after_jsb, move_group,
            planning_scene, tcp_watchdog, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true',
                              description="true => mock hardware; false => real FR5."),
        DeclareLaunchArgument('rviz', default_value=LaunchConfiguration('sim')),
        OpaqueFunction(function=_make_nodes),
    ])
