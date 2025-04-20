#!/bin/bash

while ! pgrep -f '/unitree/module/sport_mode/Legged_sport'; do
    sleep 1
done

echo "Starting control interface..."
/root/unitree_sdk2_typefly/build/bin/go2_control_interface & /root/unitree_sdk2_typefly/build/bin/go2_livox /root/unitree_sdk2_typefly/scripts/mid360_config.json &
