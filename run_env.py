#! /usr/bin/python

from dataclasses import dataclass


from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import yaml


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


def make_arm_param_list(
    span: int,
    speed: int,
    pick_time: int,
    drop_time: int,
    rest_dist: int,
    drop_dist: int,
    pos: int,
    arm_dist: int,
    count: int,
):

    base_conf = BaseArmConfig(span, speed, pick_time, drop_time)

    factory = ArmParameterFactory(
        base_conf,
        rest_dist,
        drop_dist,
        pos,
        arm_dist,
    )

    return [factory.make_robot() for _ in range(count)]


def make_arm_node(index, params):
    return Node(
        package="fake_arm",
        executable="fake_arm",
        output="screen",
        emulate_tty=True,
        name=f"fake_arm_{index}",
        parameters=[params],
    )


def initialize_arm(param_list):
    return [make_arm_node(i, param) for i, param in enumerate(param_list)]


def get_config_file():
    key = "config:="
    for arg in sys.argv:
        if arg.startswith(key):
            begin = len(key)
            value = arg[begin:]
            return value
    raise ValueError("Missing config parameter")


def set_keys(dest: dict, source: dict, keys: list, head: str):
    for k_src in keys:
        k_dst = head + "_" + k_src
        dest.setdefault(k_dst, source[k_src])


def complete_controller_config(config):
    conveior_conf = config["conveior"]
    arm_conf = config["arm"]

    controller_conf = config.get("controller", {})

    set_keys(controller_conf, conveior_conf, ["width", "length", "speed"], "conveior")

    set_keys(
        controller_conf,
        arm_conf,
        ["span", "pick_time", "drop_time", "speed", "rest_dist"],
        "arm",
    )

    config["controller"] = controller_conf
    return config


def set_robot_position(config, arm_param_list):
    pos_list = [param["robot_pos"] for param in arm_param_list]
    config["arm_pos"] = pos_list


def load_config():
    file_name = get_config_file()
    with open(file_name) as file:
        output = yaml.safe_load(file)
    return complete_controller_config(output)


def generate_launch_description():

    config = load_config()
    conveior_conf = config["conveior"]
    arm_conf = config["arm"]

    controller_conf = config.get("controller")
    arm_param = make_arm_param_list(**arm_conf)
    set_robot_position(controller_conf, arm_param)

    static_conf = [
        Node(
            package="controller",
            executable="controller",
            output="screen",
            emulate_tty=True,
            name="main_controller",
            parameters=[controller_conf],
        ),
        Node(
            package="conveior_belt",
            executable="conveior_belt",
            output="screen",
            emulate_tty=True,
            name="conveior_belt",
            parameters=[
                conveior_conf,
            ],
        ),
    ]

    arm_conf = initialize_arm(arm_param)

    launch_nodes = static_conf + arm_conf
    if config.get("gui_log"):
        gui_log = (
            Node(
                package="gui_log",
                executable="gui_log",
                output="screen",
                emulate_tty=True,
                name="gui_log",
                parameters=[{"arm_count": len(arm_param)}],
            ),
        )
        launch_nodes.append(gui_log)

    return LaunchDescription(launch_nodes)
