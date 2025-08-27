import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_ip_parameter_name_03 = 'robot_ip'    
    arm_id_parameter_name = 'arm_id'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name_03)    
    arm_id = LaunchConfiguration(arm_id_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    # 2a. gripper.launch.py에서 그리퍼 설정(YAML) 파일 경로 가져오기
    gripper_config = os.path.join(
        get_package_share_directory('franka_gripper'), 'config', 'franka_gripper_node.yaml'
    )

    # 2b. gripper.launch.py에서 'joint_names' 런치 아규먼트와 기본값 설정 가져오기
    joint_names_parameter_name = 'joint_names'
    default_joint_name_postfix = '_finger_joint'
    arm_default_argument = [
        '[',
        arm_id,
        default_joint_name_postfix,
        '1',
        ',',
        arm_id,
        default_joint_name_postfix,
        '2',
        ']',
    ]
    joint_names = LaunchConfiguration(joint_names_parameter_name)
    
    # 2c. gripper.launch.py에서 franka_gripper_node 정의 가져오기
    gripper_node = Node(
        package='franka_gripper',
        executable='franka_gripper_node',
        name=[arm_id, '_gripper'], # 노드 이름: fr3_gripper 또는 leftarm_gripper
        parameters=[{'robot_ip': robot_ip, 'joint_names': joint_names}, gripper_config],
        condition=UnlessCondition(use_fake_hardware)
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                robot_ip_parameter_name_03,
                default_value='172.16.0.3', 
                description='Hostname or IP address of the robot.'
            ),
            DeclareLaunchArgument(
                arm_id_parameter_name,
                default_value='fr3',
                description='ID of the type of arm used. Supported values: fer, fr3, fp3'
            ),
            DeclareLaunchArgument(
                use_rviz_parameter_name,
                default_value='false',
                description='Visualize the robot in Rviz',
            ),
            DeclareLaunchArgument(
                use_fake_hardware_parameter_name,
                default_value='false',
                description='Use fake hardware',
            ),
            DeclareLaunchArgument(
                fake_sensor_commands_parameter_name,
                default_value='false',
                description='Fake sensor commands. Only valid when "{}" is true'.format(
                    use_fake_hardware_parameter_name
                ),
            ),
            DeclareLaunchArgument(
                load_gripper_parameter_name,
                default_value='true',
                description=(
                    'Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                    'without an end-effector.'
                ),
            ),
            DeclareLaunchArgument(
                joint_names_parameter_name,
                default_value=arm_default_argument,
                description='Names of the gripper joints in the URDF',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare('franka_bringup'),
                             'launch', 'single_franka.launch.py']
                        )
                    ]
                ),
                launch_arguments={
                    robot_ip_parameter_name_03: robot_ip,
                    arm_id_parameter_name: arm_id,
                    load_gripper_parameter_name: load_gripper,
                    use_fake_hardware_parameter_name: use_fake_hardware,
                    fake_sensor_commands_parameter_name: fake_sensor_commands,
                    use_rviz_parameter_name: use_rviz,
                }.items(),
            ),
            GroupAction(
                actions=[
                    PushRosNamespace('leftarm'),
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        arguments=['simple_cartesian_velocity_controller'],
                        output='screen',
                    ),
                ]
            ),
            gripper_node,
        ]
    )
