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
        self.queue = queue
        self.canvas = tk.Canvas(self)

        self.prev_items = []
        self.new_items = {}

        self.update_ui()
        self.get_new_items()

    def update_ui(self):
        self.pack(fill=tk.BOTH, expand=1)
        for prev in self.prev_items:
            self.canvas.delete(prev)
        self.prev_items.clear()

        for item_x, item_y in self.new_items.values():
            new_item = self.canvas.create_oval(
                item_x - 4, item_y - 4, item_x + 4, item_y - 4, fill="#ff0000"
            )
            self.prev_items.append(new_item)
        self.new_items.clear()

        self.canvas.pack(fill=tk.BOTH, expand=1)
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
            msg.ItemLocation, "in_reach_topic", self.conveior_state_listener, 50
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
