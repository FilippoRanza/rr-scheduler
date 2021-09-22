#! /usr/bin/python3

"""
Skeleton file for a ROS node implementation
in this project.
"""

from dataclasses import dataclass
from enum import Enum, auto
import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rr_interfaces import msg
from load_config import load_configuration
from gui_log import init_gui, run_gui

from . import get_best


NODE_NAME = "controller"


def ceil(num):
    return int(np.ceil(num))


@dataclass
class ControllerConfiguration:
    """
    Controller main configuration: values loaded from
    parameters.
    """

    conveior_width: int
    conveior_length: int
    conveior_speed: int

    arm_span: int
    arm_pos: [int]
    arm_pick_time: int
    arm_drop_time: int
    arm_speed: int
    arm_rest_dist: int

class ArmState(Enum):
    """
    Current arm action.
    """

    READY = auto()
    WAITING = auto()
    WORKING = auto()

    @classmethod
    def from_int(cls, i: int):
        return [cls.READY, cls.WAITING, cls.WORKING][i]


class ArmInfo:
    """
    Manage the state of the robotic arm:
    time to reach from conveior begin and
    the current action.
    """

    def __init__(self, reach_time: int, take_time: int):
        self.time = 0
        self.min_time = reach_time + take_time
        self.state = ArmState.READY

    def set_state(self, state: ArmState):
        self.time = 0
        self.state = state

    def is_available(self, pos):
        if self.state == ArmState.READY:
            return True
        if self.state == ArmState.WAITING:
            return False
        if self.state == ArmState.WORKING:
            return self.time < self.min_time
        raise ValueError(f"ArmState is not a valid: {self.state}")


class ArmStats:
    """
    Manage statistics about the robotic arm:
    information such as the total distance and
    the number of picked items.
    """

    def __init__(self):
        self.hits = 0
        self.dist = 0
        self.temp_dist = 0

    def try_add(self, dist):
        self.temp_dist = dist
        self.dist += dist
        self.hits += 1

    def untry_add(self):
        self.dist -= self.temp_dist
        self.hits -= 1
        self.temp_dist = 0

    def add_hit(self, dist):
        self.dist += dist
        self.hits += 1


@dataclass
class ArmChooser:
    """
    Helper class: choose the best
    arm in order to:
        1. Be sure that the item will be taken
        2. Ensure load balancing amoung the various arms.
    """

    arm_stats: [ArmStats]
    arm_infos: [ArmInfo]

    def choose_best(self, pos):
        best = get_best.GetBest()
        for i, (stat, info) in self.__iter_arms__():
            if info.is_available(pos):
                val = self.run_test(stat, pos)
                print(val)
                best.update(i, val)
        return best.get_best()

    def run_test(self, arm: ArmStats, pos):
        arm.try_add(pos)
        output = self.compute_score()
        arm.untry_add()
        return output

    def compute_score(self):
        hits_var = self.get_variance(lambda arm: arm.hits)
        #dist_var = self.get_variance(lambda arm: arm.dist)
        dist_var = 0
        return hits_var + dist_var

    def get_variance(self, func):
        vec = [func(arm) for arm in self.arm_stats]
        vec = np.array(vec)
        norm = np.linalg.norm(vec)
        if norm == 0:
            norm = 1
        vec = vec / norm
        return np.var(vec)

    def __iter_arms__(self):
        return enumerate(zip(self.arm_stats, self.arm_infos))


@dataclass
class Controller:
    """
    Handle incoming request and update arm stats.
    """

    arm_stats: [ArmStats]
    arm_infos: [ArmInfo]

    def handle_new_item(self, item_pos: int):
        chooser = ArmChooser(self.arm_stats, self.arm_infos)
        best = chooser.choose_best(item_pos)
        self.arm_stats[best].add_hit(item_pos)
        self.arm_infos[best].set_state(ArmState.WAITING)
        return best

    def update_arm_state(self, state: ArmState, time: float, robot_id: int):
        self.arm_infos[robot_id].state = state
        self.arm_infos[robot_id].time = time

    def get_arm_stat_msg(self):
        output = ""
        for stat in self.arm_stats:
            curr = f"{stat.hits} {stat.dist}\t"
            output += curr
        return output

    def get_arm_info_msg(self):
        output = ""
        for info in self.arm_infos:
            curr = f"{info.state}\t"
            output += curr
        return output
        


def pythagoras(cat_a, cat_b):
    cat_a **= 2
    cat_b **= 2
    cat_c = cat_a + cat_b
    cat_c = np.sqrt(cat_c)
    return ceil(cat_c)


def compute_reach_time(conveior_speed, arm_pos, arm_span):
    min_take_dist = arm_pos - arm_span
    return ceil(min_take_dist / conveior_speed)


def compute_max_take_time(arm_speed, arm_span, rest_dist, conveior_dist):
    cathetus_a = ceil(arm_span / 2)
    cathetus_b = conveior_dist + rest_dist
    dist = pythagoras(cathetus_a, cathetus_b)
    return ceil(dist / arm_speed)


def make_arm_stat_list(count):
    return [ArmStats() for _ in range(count)]


def controller_factory(conf: ControllerConfiguration):
    max_take_time = compute_max_take_time(
        conf.arm_speed, conf.arm_span, conf.arm_rest_dist, conf.conveior_width
    )
    arm_infos = [
        ArmInfo(
            compute_reach_time(conf.conveior_speed, pos, conf.arm_span), max_take_time
        )
        for pos in conf.arm_pos
    ]
    arm_stats = make_arm_stat_list(len(arm_infos))
    return Controller(arm_stats, arm_infos)


class ControllerNode(Node):
    """Empty Node implementation"""

    def __init__(self, gui):
        """Basic constructor declaration"""
        super().__init__(NODE_NAME)
        self.config = load_configuration(self, ControllerConfiguration)
        self.controller = controller_factory(self.config)
        self.gui = gui
        self.conv_sub = self.create_subscription(
            msg.NewItem, "new_item_topic", self.conveior_state_listener, 10
        )

        self.arm_sub = self.create_subscription(
            msg.ArmState, "arm_state_topic", self.arm_state_listener, 10
        )

        self.arm_cmd = self.create_publisher(msg.TakeItem, "take_item_cmd_topic", 10)

    def conveior_state_listener(self, new_item: msg.NewItem):
        robot_id = self.controller.handle_new_item(new_item.pos)
        self.__notify_robots__(new_item.id, robot_id)
        self.gui.set_text("Status", self.controller.get_arm_stat_msg())

    def arm_state_listener(self, arm_state: msg.ArmState):
        state = ArmState.from_int(arm_state.state)
        self.controller.update_arm_state(state, arm_state.time, arm_state.robot_id)


    def __notify_robots__(self, item_id, robot_id):
        take_item = msg.TakeItem()
        take_item.item_id = item_id
        take_item.robot_id = robot_id
        self.arm_cmd.publish(take_item)


def main():
    """Default entrypoint for ros2 run"""
    rclpy.init(args=sys.argv)

    gui = init_gui("Controller", ["Status", "Info"])

    node = ControllerNode(gui)

    run_gui(gui)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
