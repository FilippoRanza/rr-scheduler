#! /usr/bin/python3

import pytest
from conveior_belt import conveior_belt


def test_spawn_rate():
    spawn_rate = conveior_belt.SpawnManager(100)
    count = 0
    for _ in range(1000):
        if spawn_rate.should_spawn():
            count += 1
    assert count == 10


def test_item_position():
    """
    Test that each item on the conveior belt is in
    the expected position
    """
    belt = init_conveior(length=1500, speed=45, steps=3)
    content = belt.content
    assert len(content) == 3
    for i in range(3):
        item = content[i]
        assert item.item_id == i

    assert belt.content[0].item_y == 90
    assert belt.content[1].item_y == 45
    assert belt.content[2].item_y == 0


def test_item_fall():
    """
    If items are not picked by robots they should 
    fall off conveior belt's end
    """
    length = 350
    speed = 50
    steps = (length // speed) + 2 # init_conveior steps and then adds a new item
    belt = init_conveior(length, speed, steps)
    # The first item is removed
    assert belt.fallen == 1, belt.content
    assert len(belt.content) == (steps - 1) 



def init_conveior(length, speed, steps):
    belt = conveior_belt.ConveiorBelt(1, 100, length)
    for _ in range(steps):
        belt.step_ahead(speed)
        belt.add_item()
    return belt
