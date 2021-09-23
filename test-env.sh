#! /bin/bash

for dir in rr_interfaces gui_log load_config controller conveior_belt fake_arm; do
    cd "$dir"
    colcon build --packages-select "$dir" &
    cd -
done

while true ; do
    count=$(jobs | wc -l)
    jobs
    if (( $count == 0 )) ; then
        break
    fi
    sleep 1
done

for dir in rr_interfaces gui_log load_config controller conveior_belt fake_arm; do
    cd "$dir"
    source install/setup.bash
    cd -
done



for dir in gui_log controller conveior_belt fake_arm; do
    cd "$dir"
    echo "$dir"
    pytest-3
    pylint "$dir"
    cd -
done



