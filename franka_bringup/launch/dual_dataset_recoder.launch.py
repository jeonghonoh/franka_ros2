from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    GroupAction, TimerAction, OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

WAYPOINT_PATH = '/home/rpm-dualarm/franka_ros2_ws/src/franka_bringup/config/joint_states.yaml'


def _controller_group(ns: str):
    """Return GroupAction that spawns dual_waypoint_step_controller under <ns>/."""
    return GroupAction(
        actions=[
            PushRosNamespace(ns),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'dual_waypoint_step_controller',
                    '--ros-args',
                    '-p', f'waypoint_file:={WAYPOINT_PATH}',
                    '-p', f'arm_id:=fr3'
                ],
                output='screen',
            ),
        ]
    )


def generate_launch_description():
    # ───────── launch‑arguments (원본 유지 + 추가) ──────────────────
    arg_defs = {
        'robot_ip_02'   : ('172.16.0.2', 'Hostname or IP of right‑base robot'),
        'robot_ip_03'   : ('172.16.0.3', 'Hostname or IP of left‑base robot'),
        'arm_id'        : ('fr3',        'Arm model (fr3)'),
        'use_rviz'      : ('false',      'Visualize in RViz'),
        'use_fake_hardware': ('false',   'Use fake hardware'),
        'fake_sensor_commands': ('false','Fake sensor cmds (needs fake hw)'),
        'load_gripper'  : ('true',       'Load Franka gripper'),
        'dataset_root'  : ('dualarm_dataset', 'Dataset output dir')
    }

    declare_args = [
        DeclareLaunchArgument(k, default_value=TextSubstitution(text=v[0]), description=v[1])
        for k, v in arg_defs.items()
    ]
    cfg = {k: LaunchConfiguration(k) for k in arg_defs}

    # ───────── launch description ───────────────────────────────
    return LaunchDescription(
        declare_args + [

        # 1) 두 대 프랑카 bring‑up (기존 Include 그대로)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('franka_bringup'), 'launch', 'dual_franka.launch.py'])),
            launch_arguments={
                'robot_ip_02': cfg['robot_ip_02'],
                'robot_ip_03': cfg['robot_ip_03'],
                'arm_id'     : cfg['arm_id'],
                'load_gripper': cfg['load_gripper'],
                'use_fake_hardware': cfg['use_fake_hardware'],
                'fake_sensor_commands': cfg['fake_sensor_commands'],
                'use_rviz'   : cfg['use_rviz'],
            }.items(),
        ),

        # 2) 왼팔·오른팔 컨트롤러 스폰
        _controller_group('leftarm'),
        _controller_group('rightarm'),

        # 3) 필요하다면, 이전 joint_trajectory_tracking_controller 스폰 유지 예시
        #    (활용하지 않을 경우 그대로 주석)
        # GroupAction(
        #     actions=[
        #         PushRosNamespace('leftarm'),
        #         Node(
        #             package='controller_manager',
        #             executable='spawner',
        #             arguments=['joint_trajectory_tracking_controller'],
        #             output='screen',
        #         ),
        #     ]
        # ),

        # 4) 데이터셋 러너 노드는 컨트롤러가 활성화된 뒤 5초 지연 후 실행
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='dual_recorder',
                    executable='dual_arm_dataset_runner',
                    output='screen',
                    parameters=[{
                        'dataset_root': cfg['dataset_root'],
                    }],
                )
            ]
        ),
    ])
