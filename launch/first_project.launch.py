import os
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('first_project')
    rviz_config = os.path.join(pkg_share, 'rviz', 'first_project.rviz')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz together with the project nodes.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use ROS simulation time, useful when replaying rosbags.',
        ),

        # ── Nodo 1: calcolo odometria ─────────────────────────────────────
        Node(
            package='first_project',
            executable='odometer',
            name='odometer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ── Nodo 2: calcolo errore TF ─────────────────────────────────────
        Node(
            package='first_project',
            executable='tf_error',
            name='tf_error',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ── RViz con configurazione progetto ──────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(use_rviz),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
