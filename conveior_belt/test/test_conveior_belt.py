#! /usr/bin/python3

import pytest
from conveior_belt import conveior_belt


def test_conveior_belt_in_reach():
    """
    Test that only the exepected object is
    considered in reach for given robot
    """
    robot = conveior_belt.Robot(0, 100, 50)
    belt = init_conveior()
    in_reach = list(belt.get_in_reach_items(robot))
    assert len(in_reach) == 1
    ((in_reach, stat),) = in_reach
    assert stat
    assert in_reach.item_id == 0
    assert in_reach.inside


def init_conveior():
    belt = conveior_belt.ConveiorBelt()
    for _ in range(3):
        belt.step_ahead(45)
        belt.add_item()
    return belt
