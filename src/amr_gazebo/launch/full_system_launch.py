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

    # 3. Launch RViz2 for 3D Tracking
    rviz_config_file = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        navigation_launch,
        rviz_node,
    ])
