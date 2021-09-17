#! /usr/bin/python

import pytest
from controller import controller


def test_empty_chooser():
    chooser = controller.ArmChooser([], [])
    with pytest.raises(ValueError):
        chooser.choose_best(0)


def test_no_arm_available():
    stats, infos = initialize_arm_list(5)
    for info in infos:
        info.set_state(controller.ArmState.WAITING)

    chooser = controller.ArmChooser(stats, infos)
    with pytest.raises(ValueError):
        chooser.choose_best(0)


def test_arm_available():
    for i in range(2, 100):
        stats, infos = initialize_arm_list(i)
        chooser = controller.ArmChooser(stats, infos)
        assert chooser.choose_best(50) == 0, f"Index: {i}"


def initialize_arm_list(count):
    stats = [controller.ArmStats() for _ in range(count)]
    infos = [controller.ArmInfo(10 + (5 * i), 5) for i in range(count)]
    for info in infos:
        assert info.state == controller.ArmState.READY
    return stats, infos
