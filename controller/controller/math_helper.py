#! /usr/bin/python

"""
Mathematical routines
to implement  controller.py
complex numerical operations.
Here manly to keep controller.py clean.
"""


import numpy as np


def fixed_norm(vec):
    """
    Return the norm of the vector.
    If the norm is zero return 1 instead
    """
    norm = np.linalg.norm(vec)
    if norm == 0:
        norm = 1
    return norm


def normed_variance(iterable):
    """
    Function takes as input an interable (a generator for example),
    converts it into a ndarray and normalize it.
    Returns normalized vector variance.
    """
    vec = list(iterable)
    np_vec = np.array(vec)
    np_vec = np_vec / fixed_norm(np_vec)
    return np.var(np_vec)


def ceil(num):
    """
    ceil function but return an interger
    """
    return int(np.ceil(num))


def pythagoras(cat_a, cat_b=None):
    """
    Compute length of hypotenuse
    knowing the length to the to cathetuses.
    THe return value is rounded to the upper integer
    """
    if cat_b is None:
        cat_b = cat_a

    cat_a **= 2
    cat_b **= 2
    cat_c = cat_a + cat_b
    cat_c = np.sqrt(cat_c)
    return ceil(cat_c)
