#! /usr/bin/python3

"""
Skeleton file for a ROS node implementation
in this project.
"""

from argparse import ArgumentParser
import random

import rclpy
from rclpy import Node
from rr_interfaces import msg


NODE_NAME = ""


class RenameMe(Node):
    """Empty Node implementation"""

    def __init__(self):
        """Basic constructor declaration"""
        super().__init__(NODE_NAME)


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
