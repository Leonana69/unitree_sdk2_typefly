#!/bin/bash

while ! pgrep -f '/unitree/module/basic_service/basic_service'; do
    sleep 1
done

echo "Starting control interface..."
/root/unitree_sdk2_typefly/build/bin/go2_control_interface &
