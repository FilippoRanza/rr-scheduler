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


@dataclass
class ConveiorConfig:
    speed: int
    width: int
    length: int
    timer_delay: float
    spawn_rate: int


@dataclass
class Item:
    item_id: int
    item_x: int
    item_y: int
    inside: bool = False

    def move_down(self, amount):
        self.item_y += amount


class SpawnManager:
    def __init__(self, spawn_rate):
        self.rate = spawn_rate
        self.index = 0

    def should_spawn(self):
        self.index = (self.index + 1) % self.rate
        return self.index == 0


class ConveiorBelt:
    def __init__(self, spawn_rate, width):
        self.curr_id = 0
        self.width = width
        self.spawn_rate = SpawnManager(spawn_rate)
        self.content = {}

    def add_item(self):
        if self.spawn_rate.should_spawn():
            pos_x = random.randint(0, self.width)
            new_id = self.__get_next_id__()
            item = Item(new_id, pos_x, 0)
            self.content[new_id] = item
            return item
        return None

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
        self.belt = ConveiorBelt(self.config.spawn_rate, self.config.width)
        self.ctrl_pub = self.create_publisher(msg.NewItem, "new_item_topic", 10)
        self.arm_pub = self.create_publisher(msg.ItemLocation, "in_reach_topic", 10)
        self.create_timer(self.config.timer_delay, self.publish_updates)

    def publish_updates(self):
        self.belt.step_ahead(self.config.speed)
        self.update_controller()
        self.send_item_location_msg()

    def update_controller(self):
        if item := self.belt.add_item():
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
    for _ in range(10000):
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
