import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the publisher node
    publisher_node = Node(
        package='bad_apple',
        executable='ascii_publisher',
        name='ascii_publisher',
        output='screen',
        parameters=[{'timer_period': 0.04}]
    )

    # Define the subscriber node
    subscriber_node = Node(
        package='bad_apple',
        executable='ascii_subscriber',
        name='ascii_subscriber',
        output='screen'
    )

    return LaunchDescription([
        publisher_node,
        subscriber_node
    ])