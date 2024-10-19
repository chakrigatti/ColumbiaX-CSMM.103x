from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    rviz_config_path = os.path.join(
         get_package_share_path('project2'),
         'rviz', 'config.rviz')
    return LaunchDescription([
        Node(
            package='project2',
            executable='publish_robot_tf'
        ),
        Node(
            package='project2',
            executable='marker_publisher'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
    ])