#! /usr/bin/python3

import pytest
from controller import get_best


class TestGetBest:
    def test_correct_usage(self):
        values = [1, 42, -2, 12, 67, 34, -12, -4, 1]
        best = get_best.GetBest()
        for i, v in enumerate(values):
            best.update(i, v)

        assert best.get_best() == 6

    def test_empty_case(self):
        best = get_best.GetBest()
        with pytest.raises(ValueError):
            best.get_best()
