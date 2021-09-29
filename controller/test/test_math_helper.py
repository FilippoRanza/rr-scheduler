#! /usr/bin/python

import numpy as np

from controller import math_helper


def test_pythagoras():
    # Correct result is an integer
    assert math_helper.pythagoras(3, 4) == 5
    assert math_helper.pythagoras(6, 8) == 10
    # Correct result is a real number
    assert math_helper.pythagoras(4, 5) == 7
    assert math_helper.pythagoras(8, 10) == 13


def test_norm():
    assert math_helper.fixed_norm([1, 4, 6]) == np.linalg.norm([1, 4, 6])
    assert math_helper.fixed_norm([0, 0, 0]) == 1
