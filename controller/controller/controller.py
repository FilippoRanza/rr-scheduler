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

from . import get_best


NODE_NAME = "controller"


class ArmState(Enum):
    READY = auto()
    WAITING = auto()
    WORKING = auto()

    @classmethod
    def from_int(cls, i: int):
        return [cls.READY, cls.WAITING, cls.WORKING][i]


class ArmInfo:
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
        dist_var = self.get_variance(lambda arm: arm.dist)
        return hits_var + dist_var

    def get_variance(self, func):
        vec = [func(arm) for arm in self.arm_stats]
        return np.var(vec)

    def __iter_arms__(self):
        return enumerate(zip(self.arm_stats, self.arm_infos))


@dataclass
class Controller:
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


class ControllerNode(Node):
    """Empty Node implementation"""

    def __init__(self, controller: Controller):
        """Basic constructor declaration"""
        super().__init__(NODE_NAME)
        self.conv_sub = self.create_subscription(
            msg.NewItem, "new_item_topic", self.conveior_state_listener, 10
        )

        self.arm_sub = self.create_subscription(
            msg.ArmState, "arm_state_topic", self.arm_state_listener, 10
        )

        self.arm_cmd = self.create_publisher(msg.TakeItem, "take_item_cmd_topic", 10)
        self.controller = controller

    def conveior_state_listener(self, new_item: msg.NewItem):
        robot_id = self.controller.handle_new_item(new_item.item_pos)
        self.__notify_robots__(new_item.item_id, robot_id)

    def arm_state_listener(self, arm_state: msg.ArmState):
        state = ArmState.from_int(arm_state.state)
        self.controller.update_arm_state(state, state.time, state.robot_id)

    def __notify_robots__(self, item_id, robot_id):
        take_item = msg.TakeItem()
        take_item.item_id = item_id
        take_item.robot_id = robot_id
        self.arm_cmd.publish(take_item)


def main():
    """Default entrypoint for ros2 run"""
    rclpy.init(args=sys.argv)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
