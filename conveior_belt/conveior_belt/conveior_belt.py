#! /usr/bin/python3

"""
Conveior Belt simulator
"""

from argparse import ArgumentParser
from dataclasses import dataclass
from enum import Enum, auto
import random

import rclpy
from rclpy.node import Node
from rr_interfaces import msg


NODE_NAME = "conveior_belt"
TIMER_PERIOD = 1 / 1000  # 1ms


@dataclass
class Robot:
    robot_id: int
    position: int
    span: int


class ItemStatus(Enum):
    IN_REACH = auto()
    OUT_REACH = auto()
    ENTERING = auto()
    EXITING = auto()

    @staticmethod
    def get_status(prev: bool, curr: bool):
        if prev:
            out = ItemStatus.__select__(curr, ItemStatus.IN_REACH, ItemStatus.EXITING)
        else:
            out = ItemStatus.__select__(curr, ItemStatus.ENTERING, ItemStatus.OUT_REACH)
        return out

    @staticmethod
    def __select__(curr: bool, t, f):
        return t if curr else f


@dataclass
class Item:
    item_id: int
    item_x: int
    item_y: int
    inside: bool = False

    def move_down(self, amount):
        self.item_y += amount

    def in_reach(self, robot: Robot):
        rel_y = abs(self.item_y - robot.position)
        stat = rel_y <= robot.span
        output = ItemStatus.get_status(self.inside, stat)
        self.inside = output
        return output


class ConveiorBelt:
    def __init__(self):
        self.id = 0
        self.content = {}

    def add_item(self):
        pos_x = random.randint(0, 100)
        new_id = self.__get_next_id__()
        item = Item(new_id, pos_x, 0)
        self.content[new_id] = item
        return item

    def take_item(self, item_id):
        self.content.pop(item_id)

    def step_ahead(self, amount):
        for item in self.content.values():
            item.move_down(amount)

    def get_in_reach_items(self, robot: Robot):
        for v in self.content.values():
            stat = v.in_reach(robot)
            if stat == ItemStatus.ENTERING:
                yield (v, True)
            elif stat == ItemStatus.EXITING:
                yield (v, False)

    def __get_next_id__(self):
        tmp = self.id
        self.id += 1
        return tmp


class ConveiorBeltNode(Node):
    """Conveior Belt Node."""

    def __init__(self, move_amout, robots: [Robot]):
        super().__init__(NODE_NAME)
        self.move_amout = move_amout
        self.robots = robots
        self.belt = ConveiorBelt()
        self.ctrl_pub = self.create_publisher(msg.NewItem, "new-item-topic", 10)
        self.arm_pub = self.create_publisher(msg.InReach, "in-reach-topic", 10)
        self.create_timer(TIMER_PERIOD, self.publish_updates)

    def publish_updates(self):
        self.belt.step_ahead(self.move_amout)
        self.update_controller()
        self.update_arms()

    def update_controller(self):
        item = self.belt.add_item()
        new_item = msg.NewItem()
        new_item.pos = item.pos_x
        new_item.id = item.item_id
        self.ctrl_pub.publish(new_item)

    def update_arms(self):
        for robot in self.robots:
            self.__send_in_reach_msg__(robot)

    def __send_in_reach_msg__(self, robot: Robot):
        for item, stat in self.belt.get_in_reach_items(robot):
            in_reach = self.__reach_msg_factory__(item, stat, robot)
            self.arm_pub.publish(in_reach)

    def __reach_msg_factory__(self, item: Item, stat: bool, robot: Robot):
        in_reach = msg.InReach()
        in_reach.state = stat
        in_reach.pos = item.pos_x
        in_reach.item_id = item.item_id
        in_reach.robot_id = robot.robot_id
        return in_reach


def parse_args():
    """Parse command line arguments"""
    parser = ArgumentParser()
    return parser.parse_args()


def main():
    """Default entrypoint for ros2 run"""
    rclpy.init()
    args = parse_args()

    node = ConveiorBeltNode()


if __name__ == "__main__":
    main()
