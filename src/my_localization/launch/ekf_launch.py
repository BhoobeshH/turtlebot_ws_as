from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='my_localization', executable='ekf_node',
             name='ekf_localizer', output='screen',
             parameters=[{'use_sim_time': True}]),
    ])