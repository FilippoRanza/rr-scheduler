#! /bin/bash

for dir in rr_interfaces gui_log gui_belt load_config controller conveior_belt fake_arm; do
    cd "$dir"
    colcon build --packages-select "$dir"
    source install/setup.bash
    cd -
done
