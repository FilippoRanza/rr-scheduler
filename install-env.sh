#! /bin/bash

set -e

function run() {
    name="$1"
    shift
    xterm -e "ros2 run $name $name $@" &disown
}


for dir in rr_interfaces controller conveior_belt fake_arm; do
    cd "$dir"
    colcon build --packages-select "$dir"
    source install/setup.bash
    cd -
done

run controller 

run conveior_belt


run fake_arm 0
run fake_arm 1
run fake_arm 2

