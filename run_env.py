#! /usr/bin/python

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="controller", executable="controller", name="main_controller"),
            Node(
                package="conveior_belt",
                executable="conveior_belt",
                name="conveior_belt",
            ),
        ]
    )
