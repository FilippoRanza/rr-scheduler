#! /usr/bin/python3


"""
Get Best: return the index
associated with the minimal value of a certain
operation. Usefull when the operation is too complex
for the max function.
"""


class GetBest:
    """To use within a for loop to identify the index with the
    associated minimal value.
    Usefull when the use case is too complex for a simple `max`
    function"""

    def __init__(self):
        self.min_value = None
        self.min_index = 0

    def update(self, index, value):
        if self.min_value is None or update_condition(self.min_value, value):
            self.min_value = value
            self.min_index = index

    def get_best(self):
        if self.min_value is None:
            raise ValueError("Current min value is None. self.update never called")
        return self.min_index


def update_condition(curr, new):
    return (curr - new) > 1e-6
