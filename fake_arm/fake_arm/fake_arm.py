#! /usr/bin/python3

"""
fake_arm.py is a simple robot arm simulator.
This script can be used to simulate an abstract pick-and-place
"""

from enum import Enum, auto
import math
import sys

import rclpy
from rclpy.node import Node
from rr_interfaces import msg


NODE_NAME = "fake_arm"
TIMER_DELAY = 1 / 1000  # 1ms

# In future version this parameters will be fetch externally
ARM_SPEED = 1

REST_POINT = (-40, 0)
DROP_POINT = (-90, 0)

PICK_TIME = 5
DROP_TIME = 3

ROBOT_POSITION = 100
ROBOT_SPAN = 100
###


class ArmState(Enum):
    READY = auto()
    WAITING = auto()
    WORKING = auto()


class ArmStatus:
    def __init__(self):
        self.time = 0
        self.state = ArmState.READY


def euclid_distance(p_1, p_2):
    x_1, y_1 = p_1
    x_2, y_2 = p_2

    d_x = (x_2 - x_1) ** 2
    d_y = (y_2 - y_1) ** 2
    dist = d_x + d_y
    return math.sqrt(dist)


def compute_work_time(pos):
    d_1 = euclid_distance(REST_POINT, pos)
    d_2 = euclid_distance(pos, DROP_POINT)
    d_3 = euclid_distance(DROP_POINT, REST_POINT)
    total = d_1 + d_2 + d_3
    time = math.ceil(total / ARM_SPEED) + DROP_TIME + PICK_TIME
    return time


def is_in_reach(item_loc, position, span):
    rel_y = abs(item_loc - position)
    return rel_y <= span


class FakeArm:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.take_items = []
        self.item_loc = {}
        self.status = ArmStatus()

    def update_state(self):
        if self.status.state == ArmState.READY:
            self.handle_ready()
        elif self.status.state == ArmState.WAITING:
            self.handle_waiting()
        elif self.status.state == ArmState.WORKING:
            self.handle_working()

    def handle_ready(self):
        if self.take_items:
            self.status.state = ArmState.WAITING

    def handle_waiting(self):
        item = self.take_items[0]
        if loc := self.item_loc.get(item):
            self.status.state = ArmState.WORKING
            self.status.time = compute_work_time(loc)
            self.take_items.pop(0)

    def handle_working(self):
        self.status.time -= 1
        if self.status.time == 0:
            if self.take_items:
                self.status.state = ArmState.WAITING
            else:
                self.status.state = ArmState.READY

    def handle_take_item(self, take_cmd: msg.TakeItem):
        if not self.is_mine(take_cmd):
            return
        item_id = take_cmd.item_id
        self.take_items.append(item_id)

    def handle_item_location(self, item_loc: msg.ItemLocation):
        if is_in_reach(item_loc.item_y, ROBOT_POSITION, ROBOT_SPAN):
            if item_loc.item_id in self.take_items:
                self.item_loc[item_loc.item_id] = (item_loc.item_x, item_loc.item_y)
        else:
            self.item_loc.pop(item_loc.item_id, None)

    def get_time(self):
        return self.status.time

    def get_state(self):
        return {ArmState.READY: 0, ArmState.WAITING: 1, ArmState.WORKING: 2}[
            self.status.state
        ]

    def is_mine(self, pkt):
        return pkt.robot_id == self.robot_id


class FakeArmNode(Node):
    """Empty Node implementation"""

    def __init__(self, arm: FakeArm):
        """Basic constructor declaration"""
        super().__init__(NODE_NAME)
        self.declare_parameter('index', 0)
        index = self.get_parameter('index').get_parameter_value().string_value
        self.__log__(f"Node index: {index}")
        self.arm = arm
        self.conv_sub = self.create_subscription(
            msg.ItemLocation, "in_reach_topic", self.conveior_belt_listener, 10
        )
        self.ctrl_sub = self.create_subscription(
            msg.TakeItem, "take_item_cmd_topic", self.controller_listener, 10
        )
        self.state_pub = self.create_publisher(msg.ArmState, "arm_state_topic", 10)
        self.create_timer(TIMER_DELAY, self.arm.update_state)

    def arm_state_broadcaster(self):
        pkt = msg.ArmState()
        pkt.robot_id = self.arm.robot_id
        pkt.time = self.arm.get_time()
        pkt.state = self.arm.get_state()
        self.state_pub.publish(pkt)

    def conveior_belt_listener(self, item_loc: msg.ItemLocation):
        self.__log__("reach info")
        self.arm.handle_item_location(item_loc)

    def controller_listener(self, take_cmd: msg.TakeItem):
        self.__log__("take item cmd")
        self.arm.handle_take_item(take_cmd)

    def __log__(self, log_msg):
        logger = self.get_logger()
        logger.info(log_msg)


def main():
    """Default entrypoint for ros2 run"""
    rclpy.init(args=sys.argv)

    fake_arm = FakeArm(0)
    node = FakeArmNode(fake_arm)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node interrupt")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
