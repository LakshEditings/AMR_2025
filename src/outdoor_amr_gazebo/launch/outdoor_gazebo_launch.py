import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('outdoor_amr_gazebo')

    urdf_file = os.path.join(pkg, 'urdf', 'outdoor_amr.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
    world_file = os.path.join(pkg, 'worlds', 'outdoor_world.world')

    return LaunchDescription([
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),

        # Gazebo server (headless physics)
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so', world_file],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='outdoor_robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
            output='screen'
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_outdoor_amr',
            arguments=[
                '-entity', 'outdoor_amr',
                '-topic',  'robot_description',
                '-x', '0', '-y', '0', '-z', '0.2',
            ],
            output='screen'
        ),

        # 3D Web UI (Outdoor)
        Node(
            package='outdoor_amr_gui',
            executable='outdoor_web_server',
            name='outdoor_web_server',
            output='screen'
        ),
    ])
