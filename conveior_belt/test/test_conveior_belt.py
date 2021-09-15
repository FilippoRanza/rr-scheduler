#! /usr/bin/python3

import pytest
from conveior_belt import conveior_belt


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


def init_conveior():
    belt = conveior_belt.ConveiorBelt()
    for _ in range(3):
        belt.step_ahead(45)
        belt.add_item()
    return belt
