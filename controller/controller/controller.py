#! /usr/bin/python3

"""
Skeleton file for a ROS node implementation
in this project.
"""

from argparse import ArgumentParser
from enum import Enum, auto

import numpy as np

import rclpy
from rclpy.node import Node
from rr_interfaces import msg


NODE_NAME = "controller"


class ArmState(Enum):
    READY = auto()
    WAITING = auto()
    WORKING = auto()


class ArmInfo:
    def __init__(self, position, span):
        self.hits = 0
        self.dist = 0
        self.time = 0
        self.state = ArmState.READY
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

    def update_time(self, time):
        if time == 0:
            self.state = ArmState.READY
        else:
            self.state = ArmState.WORKING

        self.time = time

    def is_available(self, pos):
        if self.state == ArmState.READY:
            return True
        if self.state == ArmState.WAITING:
            pass
        if self.state == ArmState.WORKING:
            pass


class GetBest:
    """To use within a for loop to identify the index with the associated minimal value. Usefull when the
    use case is too complex for a simple `max` function"""

    def __init__(self):
        self.min_value = None
        self.min_index = 0

    def update(self, index, value):
        if self.min_value is None or self.min_value > value:
            self.min_value = value
            self.min_index = index

    def get_best(self):
        if self.min_value is None:
            raise ValueError("Current min value is None. self.update never called")
        return self.min_index


class ArmChooser:
    def __init__(self, arm_count):
        self.arm_info = [ArmInfo() for _ in range(arm_count)]

    def choose_best(self, pos):
        get_best = GetBest()
        for i, arm in enumerate(self.arm_info):
            if arm.is_available():
                val = self.run_test(arm, pos)
                get_best.update(i, val)
        return get_best.get_best()

    def run_test(self, arm, pos):
        arm.try_add(pos)
        output = self.compute_score()
        arm.untry_add(pos)
        return output

    def compute_score(self):
        hits_var = self.get_variance(lambda arm: arm.hits)
        dist_var = self.get_variance(lambda arm: arm.dist)
        return hits_var + dist_var

    def get_variance(self, f):
        vec = [f(arm) for arm in self.arm_info]
        return np.var(vec)
    


class Controller:
    def __init__(self):
        pass


class ControllerNode(Node):
    """Empty Node implementation"""

    def __init__(self):
        """Basic constructor declaration"""
        super().__init__(NODE_NAME)
        self.conv_sub = self.create_subscription(
            msg.NewItem, "", self.conveior_state_listener, 10
        )

        self.arm_sub = self.create_subscription(
            msg.ArmState, "", self.arm_state_listener, 10
        )

        self.arm_cmd = self.create_publisher(msg.TakeItem, "", 10)

    def conveior_state_listener(self, msg: msg.NewItem):
        pass

    def arm_state_listener(self, msg: msg.ArmState):
        pass


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
