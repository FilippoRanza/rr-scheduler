#! /usr/bin/python3

import pytest
from controller import controller


class TestGetBest:
    def test_correct_usage(self):
        values = [1, 42, -2, 12, 67, 34, -12, -4, 1]
        get_best = controller.GetBest()
        for i, v in enumerate(values):
            get_best.update(i, v)

        assert get_best.get_best() == 6

    def test_empty_case(self):
        get_best = controller.GetBest()
        with pytest.raises(ValueError):
            get_best.get_best()
