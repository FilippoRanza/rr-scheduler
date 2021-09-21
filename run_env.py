#! /usr/bin/python

from dataclasses import dataclass


from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import yaml

ARM_SPAN = 50
ARM_SPEED = 10
ARM_PICK_TIME = 1
ARM_DROP_TIME = 1


@dataclass
class BaseArmConfig:
    span: int
    speed: int
    pick_time: int
    drop_time: int


class ArmParameterFactory:
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

    def make_robot(self):
        arm_params = {
            "arm_id": self.index,
            "arm_span": self.config.span,
            "arm_speed": self.config.speed,
            "robot_pos": self.base_pos,
            "pick_time": self.config.pick_time,
            "drop_time": self.config.drop_time,
            "rest_point": self.base_rest,
            "drop_point": self.base_drop,
        }

        self.index += 1
        self.base_pos += self.base_dist

        return arm_params


def make_arm_node(index, params):
    return Node(
        package="fake_arm",
        executable="fake_arm",
        name=f"fake_arm_{index}",
        parameters=[params],
    )


def initialize_arm(
    span: int,
    speed: int,
    pick_time: int,
    drop_time: int,
    rest_dist: int,
    drop_dist: int,
    base_pos: int,
    arm_dist: int,
    count: int,
):

    base_conf = BaseArmConfig(span, speed, pick_time, drop_time)

    factory = ArmParameterFactory(
        base_conf,
        rest_dist,
        drop_dist,
        base_pos,
        arm_dist,
    )

    return [make_arm_node(i, factory.make_robot()) for i in range(count)]


def get_config_file():
    key = "config:="
    for arg in sys.argv:
        if arg.startswith(key):
            begin = len(key)
            value = arg[begin:]
            return value
    raise ValueError("Missing config parameter")


def load_config():
    file_name = get_config_file()
    with open(file_name) as file:
        output = yaml.safe_load(file)
    return output


def generate_launch_description():

    config = load_config()
    conveior_conf = config["conveior"]
    arm_conf = config["arm"]

    controller_conf = config.get("controller", {})

    controller_conf['conveior_width'] = conveior_conf['width']
    controller_conf['conveior_length'] = conveior_conf['length']

    controller_conf['arm_span'] = arm_conf['span']
    controller_conf['arm_pos'] = arm_conf['base_pos']
    controller_conf['arm_pick_time'] = arm_conf['pick_time']
    controller_conf['arm_drop_time'] = arm_conf['drop_time']
    controller_conf['arm_speed'] = arm_conf['speed']



    static_conf = [
        Node(
            package="controller",
            executable="controller",
            name="main_controller",
            parameters=[controller_conf],
        ),
        Node(
            package="conveior_belt",
            executable="conveior_belt",
            name="conveior_belt",
            parameters=[
                conveior_conf,
            ],
        ),
    ]

    arm_conf = initialize_arm(**arm_conf)

    launch_nodes = static_conf + arm_conf

    return LaunchDescription(launch_nodes)
