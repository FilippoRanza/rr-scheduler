#! /usr/bin/python

import dataclasses

from rclpy.node import Node

MISSING_VALUE = -1


def load_configuration(node: Node, kls: type):
    create_params(node, get_fields(kls))
    values = load_params(node, get_fields(kls))
    output = kls(**values)
    log(node, output)
    return output


def create_params(node: Node, params):
    for param in params:
        node.declare_parameter(param, MISSING_VALUE)


def load_params(node: Node, params):
    return {param: load_param(node, param) for param in params}


def load_param(node, name):
    param = node.get_parameter(name)
    return param.value


def get_fields(kls: type):
    fields = dataclasses.fields(kls)
    return map(lambda field: field.name, fields)


def log(node, obj):
    logger = node.get_logger()
    log_msg = f"Load configuration for {type(node)}: {obj}"
    logger.info(log_msg)
