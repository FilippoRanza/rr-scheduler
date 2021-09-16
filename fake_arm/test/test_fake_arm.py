#! /usr/bin/python

import pytest
from fake_arm import fake_arm


def test_euclid_distance():
    """
    Test the correctness of
    the euclid distance implementation
    using Pitagorean triplets
    """

    triplettes = [
        ((0, 3), (4, 0), 5.0),
        ((4, 7), (8, 4), 5.0),  # same as above but translated by 4 on both axies
        ((0, 6), (8, 0), 10.0),
        (
            (5, 8),
            (13, 2),
            10.0,
        ),  # same as above but translated by 5 on x-axis and 2 on y-axis
    ]

    for p1, p2, dist in triplettes:
        eucl = fake_arm.euclid_distance(p1, p2)
        assert pytest.approx(eucl) == dist



def test_in_reach():
    position = 100
    span = 20
    in_out = [
        (75, False),
        (80, True),
        (90, True),
        (100, True),
        (110, True),
        (120, True),
        (125, False)
    ]

    for i, o in in_out:
        res = fake_arm.is_in_reach(i, position, span)
        assert res == o


