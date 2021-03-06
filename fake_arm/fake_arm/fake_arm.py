#! /usr/bin/python3

"""
fake_arm.py is a simple robot arm simulator.
This script can be used to simulate an abstract pick-and-place
"""

from enum import Enum, auto
from dataclasses import dataclass
import math
import sys

import rclpy
from rclpy.node import Node
from rr_interfaces import msg
from load_config import load_configuration

NODE_NAME = "fake_arm"

MISSING_VALUE = -1


@dataclass
class RobotConfig:
    arm_id: int
    arm_span: int
    arm_speed: int
    robot_pos: int
    pick_time: int
    drop_time: int
    rest_point: int
    drop_point: int
    timer_delay: float
    debug: bool


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


def compute_work_time(pos, conf: RobotConfig):

    rest_point = (conf.rest_point, conf.robot_pos)
    drop_point = (conf.drop_point, conf.robot_pos)

    d_1 = euclid_distance(rest_point, pos)
    d_2 = euclid_distance(pos, drop_point)
    d_3 = euclid_distance(drop_point, rest_point)
    total = d_1 + d_2 + d_3
    time = math.ceil(total / conf.arm_speed) + conf.drop_time + conf.pick_time
    return time


def is_in_reach(item_loc, position, span):
    rel_y = abs(item_loc - position)
    return rel_y <= span


class FakeArm:
    def __init__(self):
        self.config = None
        self.take_items = []
        self.item_loc = {}
        self.status = ArmStatus()
        self.robot_id = -1
        self.curr_item = None

    def set_config(self, conf: RobotConfig):
        self.config = conf
        self.robot_id = conf.arm_id

    def update_state(self):
        if self.status.state == ArmState.READY:
            self.handle_ready()
        elif self.status.state == ArmState.WAITING:
            self.handle_waiting()
        elif self.status.state == ArmState.WORKING:
            self.handle_working()

    def should_pick(self):
        if self.curr_item:
            output = self.curr_item
            self.curr_item = None
        else:
            output = None
        return output

    def handle_ready(self):
        if self.take_items:
            self.status.state = ArmState.WAITING

    def handle_waiting(self):
        item = self.take_items[0]
        if loc := self.item_loc.get(item):
            self.status.state = ArmState.WORKING
            self.status.time = compute_work_time(loc, self.config)
            self.curr_item = self.take_items.pop(0)

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
        if is_in_reach(item_loc.item_y, self.config.robot_pos, self.config.arm_span):
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
        return pkt.robot_id == self.config.arm_id


class FakeArmNode(Node):
    """Empty Node implementation"""

    def __init__(self, arm: FakeArm):
        """Basic constructor declaration"""
        super().__init__(NODE_NAME)
        self.config = load_configuration(self, RobotConfig)
        arm.set_config(self.config)
        self.arm = arm

        self.conv_sub = self.create_subscription(
            msg.ItemLocation, "in_reach_topic", self.conveior_belt_listener, 50
        )
        self.ctrl_sub = self.create_subscription(
            msg.TakeItem, "take_item_cmd_topic", self.controller_listener, 50
        )

        self.pick_item_pub = self.create_publisher(msg.PickItem, "pick_item_topic", 50)
        self.state_pub = self.create_publisher(msg.ArmState, "arm_state_topic", 50)
        self.queue_len_pub = self.create_publisher(
            msg.ArmQueueLen, "arm_queue_len_topic", 50
        )
        self.create_timer(self.config.timer_delay, self.run_step)

    def run_step(self):
        self.arm.update_state()
        self.arm_state_broadcaster()
        if item_id := self.arm.should_pick():
            pick_item = msg.PickItem()
            pick_item.item_id = item_id
            self.pick_item_pub.publish(pick_item)

    def arm_state_broadcaster(self):
        self.send_arm_state_packet()
        self.send_arm_queue_packet()

    def send_arm_queue_packet(self):
        pkt = msg.ArmQueueLen()
        pkt.arm_id = self.arm.robot_id
        pkt.q_len = len(self.arm.take_items)
        self.queue_len_pub.publish(pkt)

    def send_arm_state_packet(self):
        pkt = msg.ArmState()
        pkt.robot_id = self.arm.robot_id
        pkt.time = float(self.arm.get_time())
        pkt.state = self.arm.get_state()
        self.state_pub.publish(pkt)
        self.__log__(f"ID: {self.arm.robot_id} - {self.arm.get_state()}")

    def conveior_belt_listener(self, item_loc: msg.ItemLocation):
        self.arm.handle_item_location(item_loc)

    def controller_listener(self, take_cmd: msg.TakeItem):
        self.arm.handle_take_item(take_cmd)

    def is_debug(self):
        return self.config.debug

    def __log__(self, log_msg):
        logger = self.get_logger()
        logger.info(log_msg)


def run_node(node):
    if node.is_debug():
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node arrested")


def main():
    """Default entrypoint for ros2 run"""
    rclpy.init(args=sys.argv)

    fake_arm = FakeArm()
    node = FakeArmNode(fake_arm)

    run_node(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
