from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = join(
        get_package_share_directory('ekf_slam'), 'params', 'ekf_slam.yaml'
    )

    ekf_slam_node = Node(
        package='ekf_slam',
        executable='ekf_slam_node',
        name='ekf_slam_node',
        parameters=[params]
    )

    return LaunchDescription([
        ekf_slam_node
    ])
