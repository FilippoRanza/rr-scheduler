#! /usr/bin/python

from dataclasses import dataclass
import multiprocessing as mp
import tkinter as tk
import sys

import rclpy
from rclpy.node import Node
from rr_interfaces import msg
from load_config import load_configuration


@dataclass
class Config:
    length: int
    width: int


class BeltGui(tk.Frame):
    def __init__(self, master, queue):
        super().__init__(master)
        self.prev_items = []
        self.new_items = {}
        self.update_ui()
        self.get_new_items()
        self.canvas = tk.Canvas(self)

    def update_ui(self):
        for prev in self.prev_items:
            self.canvas.remove(prev)
        self.prev_items.clear()

        for x, y in self.new_items.values():
            new_item = self.canvas.create_oval(
                x - 2, y - 2, x + 2, y - 2, fill="#ff0000"
            )
            self.prev_items.append(new_item)
        self.new_items.clear()

        self.update()
        self.after(50, self.update_ui)

    def get_new_items(self):
        while not self.queue.empty():
            item_id, item_x, item_y = self.queue.get()
            self.new_items[item_id] = (item_x, item_y)
        self.after(1, self.get_new_items)


class BeltNode(Node):
    def __init__(self, queue):
        super().__init__("GUI_BELT")

        self.config = load_configuration(self, Config)
        self.queue = queue
        self.subscr = []
        self.conv_sub = self.create_subscription(
            msg.NewItem, "in_reach_topic", self.conveior_state_listener, 10
        )

    def conveior_state_listener(self, pkt: msg.ItemLocation):
        item_id = pkt.item_id
        item_x = pkt.item_x
        item_y = pkt.item_y
        data = (item_id, item_x, item_y)
        self.queue.put(data)


def main():
    rclpy.init(args=sys.argv)
    queue = mp.Queue()
    node = BeltNode(queue)

    config = node.config
    root = tk.Tk()
    root.geometry(f"{config.width}x{config.length}+0+0")
    belt_gui = BeltGui(root, queue)

    proc = mp.Process(target=belt_gui.mainloop)
    proc.start()

    rclpy.spin(node)

    proc.kill()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
