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


def test_item_position():
    """
    Test that each item on the conveior belt is in
    the expected position
    """
    belt = init_conveior()
    content = belt.content
    assert len(content) == 3
    for i in range(3):
        item = content[i]
        assert item.item_id == i

    assert belt.content[0].item_y == 90
    assert belt.content[1].item_y == 45
    assert belt.content[2].item_y == 0


def test_item_status():
    """
    Test a correct conversion of (prev_inside, curr_inside)
    into an ItemStatus enumeration
    """
    assert conveior_belt.ItemStatus.IN_REACH == conveior_belt.ItemStatus.get_status(
        True, True
    )
    assert conveior_belt.ItemStatus.OUT_REACH == conveior_belt.ItemStatus.get_status(
        False, False
    )
    assert conveior_belt.ItemStatus.ENTERING == conveior_belt.ItemStatus.get_status(
        False, True
    )
    assert conveior_belt.ItemStatus.EXITING == conveior_belt.ItemStatus.get_status(
        True, False
    )


def init_conveior():
    belt = conveior_belt.ConveiorBelt()
    for _ in range(3):
        belt.step_ahead(45)
        belt.add_item()
    return belt
