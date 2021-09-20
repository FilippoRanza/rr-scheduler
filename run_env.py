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
            Node(
                package="fake_arm",
                executable="fake_arm",
                name="fake_arm_1",
                parameters=[
                    {'index': 0}
                ]
            )

            Node(
                package="fake_arm",
                executable="fake_arm",
                name="fake_arm_2",
                parameters=[
                    {'index': 1}
                ]
            )

            Node(
                package="fake_arm",
                executable="fake_arm",
                name="fake_arm_3",
                parameters=[
                    {'index': 2}
                ]
            )
        ]
    )
