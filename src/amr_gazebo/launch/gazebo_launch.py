import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    amr_description_dir = get_package_share_directory('amr_description')
    amr_gazebo_dir = get_package_share_directory('amr_gazebo')

    # URDF via xacro
    urdf_file = os.path.join(amr_description_dir, 'urdf', 'amr_robot.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file])

    # World file
    world_file = os.path.join(amr_gazebo_dir, 'worlds', 'amr_world.world')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        # Fix for Gazebo causing Wayland crashes (mostly for gzserver if needed)
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Start Gazebo server (Headless Physics) - NO GZCLIENT!
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world_file,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),

        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_amr',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'amr_robot',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
            ],
            output='screen'
        ),
    ])
