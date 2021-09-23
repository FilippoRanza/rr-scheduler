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
    arm_count: int


class LabelAdder:
    def __init__(self, frame):
        self.row = 0
        self.frame = frame
        self.labels = {}

    def add_label(self, key):
        label = tk.Label(self.frame, text=key)
        label.grid(row=self.row, column=0, padx=20, pady=10)

        label = tk.Label(self.frame)
        label.grid(row=self.row, column=1, padx=20, pady=10)
        self.labels[key] = label

        self.row += 1

    def get_labels(self):
        return self.labels


class GuiLog(tk.Frame):
    def __init__(self, master, queue, arm_count):
        super().__init__(master)
        self.queue = queue
        self.labels = self.setup_ui(arm_count)
        self.pack()
        self.update_ui()

    def setup_ui(self, count):
        label_adder = LabelAdder(self)
        label_adder.add_label("New Item")
        for i in range(count):
            label_adder.add_label(f"Arm State {i}")
            label_adder.add_label(f"Arm Stats {i}")

        return label_adder.get_labels()

    def update_ui(self):
        while not self.queue.empty():
            self.update_label(self.queue.get())
        self.after(1, self.update_ui)

    def update_label(self, data):
        key, value = data
        lbl = self.labels[key]
        lbl["text"] = value
        self.update()


def handle_conveior_state_msg(new_item):
    data = f"id: {new_item.id} - pos: {new_item.pos}"
    return "New Item", data


def handle_arm_state_msg(arm_state):
    data = f"state: {arm_state.state} - time: {arm_state.time}"
    return f"Arm State {arm_state.robot_id}", data


def handle_arm_stats_msg(arm_stats):
    data = f"hits: {arm_stats.hits} - dist: {arm_stats.dist}"
    return f"Arm Stats {arm_stats.robot_id}", data


class LogNode(Node):
    def __init__(self, queue):
        super().__init__("LOG_NODE")

        self.config = load_configuration(self, Config)
        self.queue = queue
        self.conv_sub = self.create_subscription(
            msg.NewItem, "new_item_topic", self.conveior_state_listener, 10
        )

        self.arm_sub = self.create_subscription(
            msg.ArmState, "arm_state_topic", self.arm_state_listener, 10
        )

        self.arm_sub = self.create_subscription(
            msg.ArmStats, "controller_status_topic", self.arm_stats_listener, 10
        )

    def conveior_state_listener(self, new_item: msg.NewItem):
        data = handle_conveior_state_msg(new_item)
        self.queue.put(data)

    def arm_state_listener(self, arm_state: msg.ArmState):
        data = handle_arm_state_msg(arm_state)
        self.queue.put(data)

    def arm_stats_listener(self, arm_stats: msg.ArmStats):
        data = handle_arm_stats_msg(arm_stats)
        self.queue.put(data)

    def get_arm_count(self):
        return self.config.arm_count


def main():
    rclpy.init(args=sys.argv)
    queue = mp.Queue()
    node = LogNode(queue)

    root = tk.Tk()
    arm_count = node.get_arm_count()
    gui_log = GuiLog(root, queue, arm_count)

    proc = mp.Process(target=gui_log.mainloop)
    proc.start()

    rclpy.spin(node)

    proc.kill()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
