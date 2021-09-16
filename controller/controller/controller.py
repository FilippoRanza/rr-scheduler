#! /usr/bin/python3

"""
Skeleton file for a ROS node implementation
in this project.
"""

from argparse import ArgumentParser
from dataclasses import dataclass
from enum import Enum, auto

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

    def update_time(self, time):
        if time == 0:
            self.state = ArmState.READY
        else:
            self.state = ArmState.WORKING

        self.time = time

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


class ArmStats:
    def __init__(self):
        self.hits = 0
        self.dist = 0
        self.time = 0
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
        self.state = ArmState.WAITING


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

    def get_variance(self, f):
        vec = [f(arm) for arm in self.arm_stats]
        return np.var(vec)

    def __iter_arms__(self):
        return enumerate(zip(self.arm_stats, self.arm_infos))


@dataclass
class Controller:
    arm_stats: [ArmStats]
    arm_infos: [ArmInfo]

    def handle_new_item(self, item_pos: int, item_id: int):
        chooser = ArmChooser(self.arm_stats, self.arm_infos)
        best = chooser.choose_best(item_pos)
        self.arm_stats[best].add_hit(item_pos)
        self.arm_infos[best].set_state(ArmState.WAITING)
        return best

    


class ControllerNode(Node):
    """Empty Node implementation"""

    def __init__(self, controller: Controller):
        """Basic constructor declaration"""
        super().__init__(NODE_NAME)
        self.conv_sub = self.create_subscription(
            msg.NewItem, "", self.conveior_state_listener, 10
        )

        self.arm_sub = self.create_subscription(
            msg.ArmState, "", self.arm_state_listener, 10
        )

        self.arm_cmd = self.create_publisher(msg.TakeItem, "", 10)
        self.controller = controller


    def conveior_state_listener(self, msg: msg.NewItem):
        robot_id = self.controller.handle_new_item(msg.item_pos, msg.item_id)
        self.__notify_robots__(msg.item_id, robot_id)

    def arm_state_listener(self, msg: msg.ArmState):
        pass

    def __notify_robots__(self, item_id, robot_id):
        msg = msg.TakeItem()
        msg.item_id = item_id
        msg.robot_id = robot_id
        self.arm_cmd.publish(msg)




def parse_args():
    """Parse command line arguments"""
    parser = ArgumentParser()
    return parser.parse_args()


def main():
    """Default entrypoint for ros2 run"""
    rclpy.init()
    args = parse_args()


if __name__ == "__main__":
    main()
