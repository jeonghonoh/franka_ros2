import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from launch_ros.substitutions import FindPackageShare

import xacro

def robot_description_dependent_nodes_spawner_02(
        context: LaunchContext,
        robot_ip_02,
        arm_id,
        use_fake_hardware,
        fake_sensor_commands,
        load_gripper):

    robot_ip_str_02 = context.perform_substitution(robot_ip_02)
    arm_id_str = context.perform_substitution(arm_id)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(
        fake_sensor_commands)
    load_gripper_str = context.perform_substitution(load_gripper)

    franka_xacro_filepath = os.path.join(get_package_share_directory(
        'franka_description'), 'robots', arm_id_str, arm_id_str+'.urdf.xacro')
    robot_description = xacro.process_file(franka_xacro_filepath,
                                           mappings={
                                               'ros2_control': 'true',
                                               'arm_id': arm_id_str,
                                               'robot_ip': robot_ip_str_02,
                                               'hand': load_gripper_str,
                                               'use_fake_hardware': use_fake_hardware_str,
                                               'fake_sensor_commands': fake_sensor_commands_str,
                                           }).toprettyxml(indent='  ')

    franka_controllers = PathJoinSubstitution(
        [FindPackageShare('franka_bringup'), 'config', 'dual_controllers.yaml'])

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[franka_controllers,
                        {'robot_description': robot_description},
                        {'arm_id': arm_id},
                        {'load_gripper': load_gripper},
                        ],
            remappings=[('joint_states', 'franka/joint_states')],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            on_exit=Shutdown(),
        )
    ]

def robot_description_dependent_nodes_spawner_03(
        context: LaunchContext,
        robot_ip_03,
        arm_id,
        use_fake_hardware,
        fake_sensor_commands,
        load_gripper):

    robot_ip_str_03 = context.perform_substitution(robot_ip_03)
    arm_id_str = context.perform_substitution(arm_id)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(
        fake_sensor_commands)
    load_gripper_str = context.perform_substitution(load_gripper)

    franka_xacro_filepath = os.path.join(get_package_share_directory(
        'franka_description'), 'robots', arm_id_str, arm_id_str+'.urdf.xacro')
    robot_description = xacro.process_file(franka_xacro_filepath,
                                           mappings={
                                               'ros2_control': 'true',
                                               'arm_id': arm_id_str,
                                               'robot_ip': robot_ip_str_03,
                                               'hand': load_gripper_str,
                                               'use_fake_hardware': use_fake_hardware_str,
                                               'fake_sensor_commands': fake_sensor_commands_str,
                                           }).toprettyxml(indent='  ')

    franka_controllers = PathJoinSubstitution(
        [FindPackageShare('franka_bringup'), 'config', 'dual_controllers.yaml'])

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[franka_controllers,
                        {'robot_description': robot_description},
                        {'arm_id': arm_id},
                        {'load_gripper': load_gripper},
                        ],
            remappings=[('joint_states', 'franka/joint_states')],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            on_exit=Shutdown(),
        )
    ]

def generate_launch_description():
    arm_id_parameter_name = 'arm_id'
    # robot_ip_parameter_name_02 = 'robot_ip_02'
    robot_ip_parameter_name_03 = 'robot_ip_03'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    arm_id = LaunchConfiguration(arm_id_parameter_name)
    # robot_ip_02 = LaunchConfiguration(robot_ip_parameter_name_02)
    robot_ip_03 = LaunchConfiguration(robot_ip_parameter_name_03)    
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')

    # robot_description_dependent_nodes_spawner_opaque_function_02 = OpaqueFunction(
    #     function=robot_description_dependent_nodes_spawner_02,
    #     args=[
    #         robot_ip_02,
    #         arm_id,
    #         use_fake_hardware,
    #         fake_sensor_commands,
    #         load_gripper])
    
    robot_description_dependent_nodes_spawner_opaque_function_03 = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner_03,
        args=[
            robot_ip_03,
            arm_id,
            use_fake_hardware,
            fake_sensor_commands,
            load_gripper])

    launch_description = LaunchDescription([
        # DeclareLaunchArgument(
        #     robot_ip_parameter_name_02,
        #     default_value='172.16.0.2',
        #     description='Hostname or IP address of the robot.'),
        DeclareLaunchArgument(
            robot_ip_parameter_name_03,
            default_value='172.16.0.3',
            description='Hostname or IP address of the robot.'),
        DeclareLaunchArgument(
            arm_id_parameter_name,
            default_value='fr3',
            description='ID of the type of arm used. Supported values: fer, fr3, fp3'),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='false',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description='Fake sensor commands. Only valid when "{}" is true'.format(
                use_fake_hardware_parameter_name)),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        
        # 172.16.0.2, rightarm group
        # GroupAction(
        #     actions=[
        #         PushRosNamespace('rightarm'),
        #         Node(
        #             package='joint_state_publisher',
        #             executable='joint_state_publisher',
        #             name='joint_state_publisher',
        #             parameters=[
        #                 {'source_list': ['franka/joint_states', 'franka_gripper/joint_states'],
        #                 'rate': 30}],
        #         ),
        #         robot_description_dependent_nodes_spawner_opaque_function_02,
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource([PathJoinSubstitution(
        #                 [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        #             launch_arguments={'robot_ip': robot_ip_02,
        #                             use_fake_hardware_parameter_name: use_fake_hardware}.items(),
        #             condition=IfCondition(load_gripper)
        #         ),
        #         Node(
        #             package='controller_manager',
        #             executable='spawner',
        #             arguments=['joint_state_broadcaster'],
        #             output='screen',
        #         ),
        #         Node(
        #             package='controller_manager',
        #             executable='spawner',
        #             arguments=['franka_robot_state_broadcaster',],
        #             parameters=[{'arm_id': arm_id}],
        #             output='screen',
        #             condition=UnlessCondition(use_fake_hardware),
        #         ),
        #         Node(package='rviz2',
        #             executable='rviz2',
        #             name='rviz2',
        #             arguments=['--display-config', rviz_file],
        #             condition=IfCondition(use_rviz)
        #         ),
        #     ]
        # ),
        
        # 172.16.0.3, leftarm group
        GroupAction(
            actions=[
                PushRosNamespace('leftarm'),
                Node(
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                    parameters=[
                        {'source_list': ['franka/joint_states', 'franka_gripper/joint_states'],
                        'rate': 30}],
                ),
                robot_description_dependent_nodes_spawner_opaque_function_03,
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([PathJoinSubstitution(
                        [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
                    launch_arguments={'robot_ip': robot_ip_03,
                                    use_fake_hardware_parameter_name: use_fake_hardware}.items(),
                    condition=IfCondition(load_gripper)
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen',
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['franka_robot_state_broadcaster',],
                    parameters=[{'arm_id': arm_id}],
                    output='screen',
                    condition=UnlessCondition(use_fake_hardware),
                ),
                Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['--display-config', rviz_file],
                    condition=IfCondition(use_rviz)
                ),
            ]
        )
        
        
        
    ])

    return launch_description
