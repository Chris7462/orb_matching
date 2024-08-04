from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    orb_matching_node = Node(
        package='orb_matching',
        executable='orb_matching_node',
        name='orb_matching_node',
        output='screen'
    )

    return LaunchDescription([
        orb_matching_node
    ])
