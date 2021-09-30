#! /usr/bin/python

"""
Skeleton file for a ROS node implementation
in this project.
"""

from dataclasses import dataclass
from enum import Enum, auto
import sys

import rclpy
from rclpy.node import Node
from rr_interfaces import msg
from load_config import load_configuration

from . import get_best
from . import math_helper

NODE_NAME = "controller"


@dataclass
class ControllerConfiguration:
    """
    Controller main configuration: values loaded from
    parameters.
    """

    conveior_width: int
    conveior_length: int
    conveior_speed: int

    arm_span: int
    arm_pos: [int]
    arm_pick_time: int
    arm_drop_time: int
    arm_speed: int
    arm_rest_dist: int

    timer_delay: float

    debug: bool


@dataclass
class TakeTime:
    arm_speed: int
    arm_span: int
    rest_dist: int

    def get_max_time(self, dist):
        cathetus_a = self.arm_span
        cathetus_b = dist + self.rest_dist
        dist = math_helper.pythagoras(cathetus_a, cathetus_b)
        return math_helper.ceil(dist / self.arm_speed)


class ArmState(Enum):
    """
    Current arm action.
    """

    READY = auto()
    WAITING = auto()
    WORKING = auto()

    @classmethod
    def from_int(cls, i: int):
        return [cls.READY, cls.WAITING, cls.WORKING][i]


@dataclass
class LastArmItem:
    arm_id: int
    curr_loc: int


class ArmInfo:
    """
    Manage the state of the robotic arm:
    time to reach from conveior begin and
    the current action.
    """

    def __init__(
        self,
        reach_time: int,
        take_time: TakeTime,
        limit: int,
        conveior_speed: int,
    ):
        self.reach_time = reach_time
        self.take_time = take_time
        self.last_item = None
        self.limit = limit
        self.conveior_speed = conveior_speed

    def set_state(self, item_id: int):
        self.last_item = item_id

    def is_available(self, pos, cache_dict):
        if self.last_item is None:
            return True
        return self.check_time(pos, cache_dict)

    def check_time(self, pos, cache_dict):
        time = self.time_for_last_item(pos, cache_dict)
        return time < self.reach_time

    def time_for_last_item(self, pos, cache_dict):
        take_time = self.take_time.get_max_time(pos)
        if last_item := cache_dict.get(self.last_item):
            dist = self.limit - last_item.curr_loc
            time = math_helper.ceil(dist / self.conveior_speed)
            return time + take_time
        return take_time + self.reach_time


class ArmStats:
    """
    Manage statistics about the robotic arm:
    information such as the total distance and
    the number of picked items.
    """

    def __init__(self):
        self.hits = 0
        self.dist = 0
        self.temp_dist = 0

    def try_add(self, dist):
        self.temp_dist = dist
        self.dist += dist
        self.hits += 1

    def untry_add(self):
        self.dist -= self.temp_dist
        self.hits -= 1
        self.temp_dist = 0

    def add_hit(self, dist):
        self.dist += dist
        self.hits += 1


@dataclass
class ArmChooser:
    """
    Helper class: choose the best
    arm in order to:
        1. Be sure that the item will be taken
        2. Ensure load balancing amoung the various arms.
    """

    arm_stats: [ArmStats]
    arm_infos: [ArmInfo]
    item_cache: dict

    def choose_best(self, pos):
        best = get_best.GetBest()
        for i, (stat, info) in self.__iter_arms__():
            if info.is_available(pos, self.item_cache):
                val = self.run_test(stat, pos)
                best.update(i, val)
        return best.get_best()

    def run_test(self, arm: ArmStats, pos):
        arm.try_add(pos)
        output = self.compute_score()
        arm.untry_add()
        return output

    def compute_score(self):
        hits_var = self.get_score(lambda arm: arm.hits)
        dist_var = self.get_score(lambda arm: arm.dist)
        return hits_var + dist_var

    def get_score(self, func):
        return math_helper.normed_variance(func(arm) for arm in self.arm_stats)

    def __iter_arms__(self):
        return enumerate(zip(self.arm_stats, self.arm_infos))


@dataclass
class Controller:
    """
    Handle incoming request and update arm stats.
    """

    arm_stats: [ArmStats]
    arm_infos: [ArmInfo]
    item_cache: dict

    def handle_new_item(self, item_id: int, item_pos: int):
        chooser = ArmChooser(self.arm_stats, self.arm_infos, self.item_cache)
        best = chooser.choose_best(item_pos)
        self.arm_stats[best].add_hit(item_pos)
        self.arm_infos[best].set_state(item_id)
        self.item_cache[item_id] = LastArmItem(best, item_pos)

        return best

    def update_arm_state(self, state: ArmState, robot_id: int):
        if state == ArmState.READY:
            self.arm_infos[robot_id].last_item = None

    def get_arm_stat_msg(self):
        output = ""
        for stat in self.arm_stats:
            curr = f"{stat.hits} {stat.dist}\t"
            output += curr
        return output

    def get_arm_info_msg(self):
        output = ""
        for info in self.arm_infos:
            curr = f"{info.state}\t"
            output += curr
        return output

    def remove_item(self, item_id):
        self.item_cache.pop(item_id)

    def update_location(self, item_id, item_pos):
        if item := self.item_cache.get(item_id):
            item.curr_loc = item_pos


def compute_reach_time(conveior_speed, arm_pos, arm_span):
    min_take_dist = arm_pos - arm_span
    return math_helper.ceil(min_take_dist / conveior_speed)


def compute_max_take_time(arm_speed, arm_span, rest_dist, conveior_dist):
    cathetus_a = arm_span
    cathetus_b = conveior_dist + rest_dist
    dist = math_helper.pythagoras(cathetus_a, cathetus_b)
    return math_helper.ceil(dist / arm_speed)


def make_arm_stat_list(count):
    return [ArmStats() for _ in range(count)]


def controller_factory(conf: ControllerConfiguration):
    take_time = TakeTime(conf.arm_speed, conf.arm_span, conf.rest_dist)
    arm_infos = [
        ArmInfo(
            compute_reach_time(conf.conveior_speed, pos, conf.arm_span),
            take_time,
            pos - conf.arm_span,
            conf.conveior_speed,
        )
        for pos in conf.arm_pos
    ]
    arm_stats = make_arm_stat_list(len(arm_infos))
    return Controller(arm_stats, arm_infos, {})


class ControllerNode(Node):
    """Empty Node implementation"""

    def __init__(self):
        """Basic constructor declaration"""
        super().__init__(NODE_NAME)
        self.config = load_configuration(self, ControllerConfiguration)
        self.controller = controller_factory(self.config)
        self.conv_sub = self.create_subscription(
            msg.NewItem, "new_item_topic", self.conveior_state_listener, 50
        )

        self.arm_sub = self.create_subscription(
            msg.ArmState, "arm_state_topic", self.arm_state_listener, 50
        )

        self.pick_item_sub = self.create_subscription(
            msg.PickItem, "pick_item_topic", self.pick_item_listener, 50
        )

        self.arm_cmd = self.create_publisher(msg.TakeItem, "take_item_cmd_topic", 50)

        self.stat_pub = self.create_publisher(
            msg.ArmStats, "controller_status_topic", 50
        )

        self.item_loc_sub = self.create_subscription(
            msg.ItemLocation, "in_reach_topic", self.item_loc_listener, 50
        )

        self.create_timer(self.config.timer_delay, self.send_controller_status)

    def item_loc_listener(self, item_loc: msg.ItemLocation):
        self.controller.update_location(item_loc.item_id, item_loc.item_y)

    def pick_item_listener(self, pick_item: msg.PickItem):
        self.controller.remove_item(pick_item.item_id)

    def conveior_state_listener(self, new_item: msg.NewItem):
        robot_id = self.controller.handle_new_item(new_item.id, new_item.pos)
        self.__notify_robots__(new_item.id, robot_id)

    def arm_state_listener(self, arm_state: msg.ArmState):
        state = ArmState.from_int(arm_state.state)
        self.controller.update_arm_state(state, arm_state.robot_id)

    def send_controller_status(self):
        for i, arm in enumerate(self.controller.arm_stats):
            arm_stats = msg.ArmStats()
            arm_stats.robot_id = i
            arm_stats.hits = arm.hits
            arm_stats.dist = arm.dist
            self.stat_pub.publish(arm_stats)
            self.get_logger().info(f"STAT: {arm_stats}")

    def is_debug(self):
        return self.config.debug

    def __notify_robots__(self, item_id, robot_id):
        take_item = msg.TakeItem()
        take_item.item_id = item_id
        take_item.robot_id = robot_id
        self.arm_cmd.publish(take_item)


def main():
    """Default entrypoint for ros2 run"""
    rclpy.init(args=sys.argv)

    node = ControllerNode()

    if not node.is_debug():
        rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
