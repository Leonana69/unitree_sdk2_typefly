#!/bin/bash

echo "Starting lidar service..."
/root/unitree_sdk2_typefly/build/bin/go2_livox /root/unitree_sdk2_typefly/scripts/mid360_config.json &
