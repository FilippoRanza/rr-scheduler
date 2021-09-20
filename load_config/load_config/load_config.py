#! /usr/bin/python

import dataclasses

from rclpy.node import Node

MISSING_VALUE = -1


def load_config(node: Node, kls: type):
    create_params(node, get_fields(kls))
    values = load_params(node, get_fields(kls))
    return kls(**values)


def create_params(node: Node, params):
    for param in params:
        node.create_parameter(param, MISSING_VALUE)


def load_params(node: Node, params):
    return {param: load_param(node, param) for param in params}


def load_param(node, name):
    value = self.get_parameter(name).get_parameter_value().integer_value
    if value == MISSING_VALUE:
        raise ValueError(f"Parameter `{name}` is not set")
    return value


def get_fields(kls: type):
    fields = dataclasses.fields(kls)
    return map(lambda field: field.name, fields)
