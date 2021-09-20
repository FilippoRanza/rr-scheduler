#! /bin/bash

set -e

function run() {
    name="$1"
    shift
    xterm -e "ros2 run $name $name $@" &disown
}


for dir in rr_interfaces load_config controller conveior_belt fake_arm; do
    cd "$dir"
    colcon build --packages-select "$dir"
    source install/setup.bash
    cd -
done

ros2 launch run_env.py

# run controller 

# run conveior_belt


# run fake_arm 0
# run fake_arm 1
# run fake_arm 2

