from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_parameter_name_02 = 'robot_ip_02'
    robot_ip_parameter_name_03 = 'robot_ip_03'
    arm_id_parameter_name = 'arm_id'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    # LaunchConfigurations
    robot_ip_02 = LaunchConfiguration(robot_ip_parameter_name_02)
    robot_ip_03 = LaunchConfiguration(robot_ip_parameter_name_03)
    arm_id = LaunchConfiguration(arm_id_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            robot_ip_parameter_name_02,
            default_value='172.16.0.2',
            description='Hostname or IP address of the first robot (02).'
        ),
        DeclareLaunchArgument(
            robot_ip_parameter_name_03,
            default_value='172.16.0.3',
            description='Hostname or IP address of the second robot (03).'
        ),
        DeclareLaunchArgument(
            arm_id_parameter_name,
            default_value='fr3',
            description='Type of arm. (fr3, etc.)'
        ),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper or not.'
        ),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware or not.'
        ),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description='Fake sensor commands (valid if use_fake_hardware is true).'
        ),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='false',
            description='Visualize the robot in Rviz'
        ),

        # ---single bringup ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution(
                    [FindPackageShare('franka_bringup'), 'launch', 'single_franka_02.launch.py']
                )
            ]),
            launch_arguments={
                'robot_ip_02': robot_ip_02,
                'arm_id': arm_id,
                'load_gripper': load_gripper,
                'use_fake_hardware': use_fake_hardware,
                'fake_sensor_commands': fake_sensor_commands,
                'use_rviz': use_rviz
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution(
                    [FindPackageShare('franka_bringup'), 'launch', 'single_franka_03.launch.py']
                )
            ]),
            launch_arguments={
                'robot_ip_03': robot_ip_03,
                'arm_id': arm_id,
                'load_gripper': load_gripper,
                'use_fake_hardware': use_fake_hardware,
                'fake_sensor_commands': fake_sensor_commands,
                'use_rviz': use_rviz
            }.items(),
        ),


        # --- MoveToStart controller (02) ---
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_manager_02',
            arguments=[
                'move_to_start_example_controller_02',
                '--controller-manager', 'controller_manager_02'
            ],
            output='screen',
        ),

        # --- MoveToStart controller (03) ---
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_manager_03',
            arguments=[
                'move_to_start_example_controller_03',
                '--controller-manager', 'controller_manager_03'
            ],
            output='screen',
        ),
    ])
