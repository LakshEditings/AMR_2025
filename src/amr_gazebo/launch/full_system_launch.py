import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    amr_gazebo_dir = get_package_share_directory('amr_gazebo')
    amr_navigation_dir = get_package_share_directory('amr_navigation')

    # 1. Launch Gazebo with robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_gazebo_dir, 'launch', 'gazebo_launch.py')
        )
    )

    # 2. Launch Navigation (delayed to let Gazebo start first)
    navigation_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(amr_navigation_dir, 'launch', 'navigation_launch.py')
                )
            )
        ]
    )

    # 3. Launch Custom 3D Web UI (Replaces RViz)
    web_server_node = Node(
        package='amr_gui',
        executable='amr_web_server',
        name='amr_web_server',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        navigation_launch,
        web_server_node,
    ])
