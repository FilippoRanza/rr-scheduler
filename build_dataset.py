#! /usr/bin/python

from argparse import ArgumentParser
from dataclasses import dataclass
import random
import os
import subprocess
import sqlite3

import yaml


@dataclass
class ValueConfig:
    min_value: int
    max_value: int

    def rand_int(self, upper=None, lower=None):
        if self.min_value == self.max_value:
            return self.min_value

        if upper is None:
            upper = self.max_value
        if lower is None:
            lower = self.min_value

        if lower < 0 and upper < 0:
            lower = abs(lower)
            upper = abs(upper)
            return -random.randint(lower, upper)

        return random.randint(lower, upper)


@dataclass
class ArmInstance:
    span: int
    speed: int
    pick_time: int
    drop_time: int
    rest_dist: int
    drop_dist: int
    pos: int
    arm_dist: int
    count: int


@dataclass
class ArmConfig:
    span: ValueConfig
    speed: ValueConfig
    pick_time: ValueConfig
    drop_time: ValueConfig
    rest_dist: ValueConfig
    drop_dist: ValueConfig
    pos: ValueConfig
    arm_dist: ValueConfig
    count: ValueConfig

    def into_instance(self):
        arm_span = self.span.rand_int()
        arm_speed = self.speed.rand_int()
        arm_pick_time = self.pick_time.rand_int()
        arm_drop_time = self.drop_time.rand_int()
        arm_rest_dist = self.rest_dist.rand_int()
        arm_drop_dist = self.drop_dist.rand_int(lower=arm_rest_dist)
        arm_pos = self.pos.rand_int()
        arm_arm_dist = self.arm_dist.rand_int(lower=2 * arm_span)
        arm_count = self.count.rand_int()
        return ArmInstance(
            span=arm_span,
            speed=arm_speed,
            pick_time=arm_pick_time,
            drop_time=arm_drop_time,
            rest_dist=arm_rest_dist,
            drop_dist=arm_drop_dist,
            pos=arm_pos,
            arm_dist=arm_arm_dist,
            count=arm_count,
        )


@dataclass
class ConveiorInstance:
    speed: int
    width: int
    length: int
    spawn_rate: int


@dataclass
class ConveiorConf:
    speed: ValueConfig
    width: ValueConfig
    length: ValueConfig
    spawn_rate: ValueConfig

    def into_instance(self, arm_inst: ArmInstance):
        conv_speed = self.speed.rand_int()
        conv_width = self.width.rand_int()
        conv_length = self.length.rand_int(
            lower=arm_inst.count * arm_inst.pos + arm_inst.span
        )
        conv_spawn = self.spawn_rate.rand_int()
        return ConveiorInstance(
            speed=conv_speed,
            width=conv_width,
            length=conv_length,
            spawn_rate=conv_spawn,
        )


@dataclass
class Configuration:
    arm_conf: ArmConfig
    conv_conf: ConveiorConf


def convert_dict(conf_dict: dict):
    return {k: ValueConfig(**v) for k, v in conf_dict.items()}


def load_config(yaml_file):
    with open(yaml_file) as file:
        config = yaml.safe_load(file)

    arm_conf = ArmConfig(**convert_dict(config["arm_conf"]))
    conv_conf = ConveiorConf(**convert_dict(config["conv_conf"]))

    return Configuration(arm_conf, conv_conf)


def create_database(name):
    conn = sqlite3.connect(name)
    curs = conn.cursor()
    create_instance_table = """
        CREATE TABLE instances (
            id integer,
            conv_speed integer,
            conv_width integer,
            conv_length integer,
            conv_spawn_rate integer,
            arm_span integer,
            arm_speed integer,
            arm_pick_time integer,
            arm_drop_time integer,
            arm_rest_dist integer,
            arm_drop_dist integer,
            arm_pos integer,
            arm_arm_dist integer,
            arm_count integer,
            status integer
        )
    """
    curs.execute(create_instance_table)
    create_count_table = """
        CREATE TABLE instanceCounter (curr integer)
    """
    curs.execute(create_count_table)
    initialize_count = """
        INSERT INTO instanceCounter (curr) VALUES (0)
    """
    curs.execute(initialize_count)
    conn.commit()
    conn.close()


def insert_instance(
    inst_id: int, arm_inst: ArmInstance, conv_inst: ConveiorInstance, db_name
):
    conn = open_connection(db_name)
    curs = conn.cursor()
    query = f"""
    INSERT INTO instances(
        id,
        conv_speed,
        conv_width,
        conv_length,
        conv_spawn_rate,
        arm_span,
        arm_speed,
        arm_pick_time,
        arm_drop_time,
        arm_rest_dist,
        arm_drop_dist,
        arm_pos,
        arm_arm_dist,
        arm_count,
        status
    ) VALUES (
        {inst_id},
        {conv_inst.speed},
        {conv_inst.width},
        {conv_inst.length},
        {conv_inst.spawn_rate},
        {arm_inst.span},
        {arm_inst.speed},
        {arm_inst.pick_time},
        {arm_inst.drop_time},
        {arm_inst.rest_dist},
        {arm_inst.drop_dist},
        {arm_inst.pos},
        {arm_inst.arm_dist},
        {arm_inst.count},
        0
    )
    """
    curs.execute(query)
    conn.commit()
    conn.close()


def get_next_index(db_name):
    conn = conn = open_connection(db_name)
    curs = conn.cursor()
    query = """
    SELECT curr FROM instanceCounter
    """
    res = curs.execute(query)
    (index,) = next(res)
    query = f"""
    UPDATE instanceCounter 
    SET curr = {index + 1}
    WHERE curr = {index}
    """
    curs.execute(query)
    conn.commit()
    conn.close()
    return index


def open_connection(name):
    if not os.path.isfile(name):
        create_database(name)

    conn = sqlite3.connect(name)
    return conn


def make_config_file(database: str, 
    config_name: str, index: int, arm_conf: ArmConfig, conv_conf: ConveiorConf
):

    conv_conf = vars(conv_conf)
    conv_conf['spawn_count'] = 180
    config = {
        "database": database,
        "index": index,
        "arm": vars(arm_conf),
        "conveior": conv_conf,
        "timer-delay": 0.005,
    }
    with open(config_name, "w") as file:
        yaml.safe_dump(config, file)


def run_docker():
    wd = os.getcwd()
    subprocess.run(["docker", "run", "-v", f"{wd}:/root", "run-ros"])


def parse_args():
    parser = ArgumentParser()

    parser.add_argument("config")
    parser.add_argument("count", type=int)
    parser.add_argument("database")
    parser.add_argument("config_name")

    return parser.parse_args()


def main():
    args = parse_args()
    conf = load_config(args.config)
    for _ in range(args.count):
        arm_inst = conf.arm_conf.into_instance()
        conv_inst = conf.conv_conf.into_instance(arm_inst)
        index = get_next_index(args.database)
        insert_instance(index, arm_inst, conv_inst, args.database)
        make_config_file(args.database, args.config_name, index, arm_inst, conv_inst)
        run_docker()

if __name__ == "__main__":
    main()
