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
    spawn_count: int
    debug: bool


@dataclass
class Item:
    item_id: int
    item_x: int
    item_y: int
    inside: bool = False

    def move_down(self, amount):
        self.item_y += amount


class SpawnManager:
    def __init__(self, spawn_rate, spawn_count):
        self.rate = spawn_rate
        self.count = spawn_count
        self.index = 0

    def should_spawn(self):
        self.index = (self.index + 1) % self.rate
        status = self.index == 0
        if status:
            self.count -= 1

        return status if self.count >= 0 else False


class ConveiorBelt:
    def __init__(self, spawn_manager, width, length):
        self.curr_id = 0
        self.width = width
        self.lengh = length
        self.spawn_rate = spawn_manager
        self.content = {}
        self.fallen = 0

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
        fallen = []
        for index, item in self.content.items():
            item.move_down(amount)
            if item.item_y > self.lengh:
                fallen.append(index)

        self.fallen += len(fallen)
        for fall in fallen:
            self.content.pop(fall)

    def __get_next_id__(self):
        tmp = self.curr_id
        self.curr_id += 1
        return tmp


class ConveiorBeltNode(Node):
    """Conveior Belt Node."""

    def __init__(self):
        super().__init__(NODE_NAME)
        self.config = load_configuration(self, ConveiorConfig)
        spawn_manager = SpawnManager(self.config.spawn_rate, self.config.spawn_count)
        self.belt = ConveiorBelt(spawn_manager, self.config.width, self.config.length)
        self.ctrl_pub = self.create_publisher(msg.NewItem, "new_item_topic", 50)
        self.arm_pub = self.create_publisher(msg.ItemLocation, "in_reach_topic", 50)
        self.count_pub = self.create_publisher(msg.ItemCount, "item_count_topic", 50)

        self.pick_sub = self.create_subscription(
            msg.PickItem, "pick_item_topic", self.pick_item_listener, 50
        )

        self.create_timer(self.config.timer_delay, self.publish_updates)

    def pick_item_listener(self, pick_item: msg.PickItem):
        self.belt.take_item(pick_item.item_id)

    def publish_updates(self):
        self.belt.step_ahead(self.config.speed)
        self.update_controller()
        self.send_item_location_msg()
        self.send_item_count_msg()

    def send_item_count_msg(self):
        count = len(self.belt.content)
        item_count = msg.ItemCount()
        item_count.count = count
        self.count_pub.publish(item_count)

    def update_controller(self):
        if item := self.belt.add_item():
            self.get_logger().info(f"ITEM: {item}")
            new_item = msg.NewItem()
            new_item.pos = item.item_x
            new_item.id = item.item_id
            self.ctrl_pub.publish(new_item)

    def send_item_location_msg(self):
        for item in self.belt.content.values():
            item_loc = reach_msg_factory(item)
            self.arm_pub.publish(item_loc)

    def is_debug(self):
        return self.config.debug

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
    if not node.is_debug():
        rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
