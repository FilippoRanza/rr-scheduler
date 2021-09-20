#! /usr/bin/python

from dataclasses import dataclass

from launch import LaunchDescription
from launch_ros.actions import Node


@dataclass
class BaseArmConfig:
    span: int
    speed: int
    pick_time: int
    drop_time: int


class ArmArmInitializer:
    def __init__(
        self,
        base_conf: BaseArmConfig,
        base_rest: int,
        base_drop: int,
        base_pos: int,
        base_dist: int,
    ):
        self.index = 0
        self.config = base_conf
        self.base_pos = base_pos
        self.base_rest = base_rest
        self.base_drop = base_drop
        self.base_dist = base_dist

    def make_robots(self, count):
        return [self.make_robot() for _ in range(count)]

    def make_robot(self):
        node_conf = Node(
            package="fake_arm",
            executable="fake_arm",
            name=f"fake_arm_{self.index}",
            parameters=[
                {
                    "arm_id": self.index,
                    "arm_span": self.config.span,
                    "arm_speed": self.config.speed,
                    "robot_pos": self.base_pos,
                    "pick_time": self.config.pick_time,
                    "drop_time": self.config.drop_time,
                    "rest_point": self.base_rest,
                    "drop_point": self.base_drop,
                }
            ],
        )

        self.index += 1
        self.base_pos += self.base_dist

        return node_conf


def generate_launch_description():

    static_conf = [
        Node(package="controller", executable="controller", name="main_controller"),
        Node(
            package="conveior_belt",
            executable="conveior_belt",
            name="conveior_belt",
        ),
    ]

    arm_config = BaseArmConfig(50, 10, 1, 1)
    arm_generator = ArmArmInitializer(arm_config, -10, -40, 100, 250)
    arm_conf = arm_generator.make_robots(3)

    launch_nodes = static_conf + arm_conf

    return LaunchDescription(launch_nodes)
