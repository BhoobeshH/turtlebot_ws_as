from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='my_localization', executable='noisy_odom',
             name='noisy_odom', output='screen',
             parameters=[{'use_sim_time': True,
                          'v_scale_err': 0.05,
                          'curv_bias': 0.05}]),
        Node(package='my_localization', executable='ekf_node',
             name='ekf_localizer', output='screen',
             parameters=[{'use_sim_time': True,
                          'q_xy': 0.001, 'q_yaw': 0.02, 'r_yaw': 0.05}],
             remappings=[('/odom', '/odom_noisy')]),
    ])