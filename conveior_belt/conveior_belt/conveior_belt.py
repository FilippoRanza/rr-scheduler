#! /usr/bin/python3

"""
Conveior Belt simulator
"""

from dataclasses import dataclass
import random
import sys

import rclpy
from rclpy.node import Node
from rr_interfaces import msg
from load_config import load_configuration


NODE_NAME = "conveior_belt"
TIMER_PERIOD = 1 / 1000  # 1ms


@dataclass
class ConveiorConfig:
    conveior_speed: int


@dataclass
class Item:
    item_id: int
    item_x: int
    item_y: int
    inside: bool = False

    def move_down(self, amount):
        self.item_y += amount


class ConveiorBelt:
    def __init__(self):
        self.curr_id = 0
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

    def __get_next_id__(self):
        tmp = self.curr_id
        self.curr_id += 1
        return tmp


class ConveiorBeltNode(Node):
    """Conveior Belt Node."""

    def __init__(self):
        super().__init__(NODE_NAME)
        self.config = load_configuration(self, ConveiorConfig)
        self.belt = ConveiorBelt()
        self.ctrl_pub = self.create_publisher(msg.NewItem, "new_item_topic", 10)
        self.arm_pub = self.create_publisher(msg.ItemLocation, "in_reach_topic", 10)
        self.create_timer(TIMER_PERIOD, self.publish_updates)

    def publish_updates(self):
        self.belt.step_ahead(self.move_amout)
        self.update_controller()
        self.send_item_location_msg()

    def update_controller(self):
        item = self.belt.add_item()
        new_item = msg.NewItem()
        new_item.pos = item.item_x
        new_item.id = item.item_id
        self.ctrl_pub.publish(new_item)

    def send_item_location_msg(self):
        for item in self.belt.content.values():
            item_loc = reach_msg_factory(item)
            self.arm_pub.publish(item_loc)


def reach_msg_factory(item: Item):
    in_reach = msg.ItemLocation()
    in_reach.item_x = item.item_x
    in_reach.item_y = item.item_y
    in_reach.item_id = item.item_id
    return in_reach


def main():
    """Default entrypoint for ros2 run"""
    rclpy.init(args=sys.argv)

    node = ConveiorBeltNode()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
