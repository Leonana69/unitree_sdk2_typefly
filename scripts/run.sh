#!/bin/bash

set -a
source .env
set +a

/root/unitree_sdk2_typefly/scripts/video_forward.sh &
/root/unitree_sdk2_typefly/scripts/livox_service.sh &
/root/unitree_sdk2_typefly/scripts/control_interface.sh &
/root/unitree_sdk2_typefly/scripts/audio_forward.sh &
/root/unitree_sdk2_typefly/scripts/high_state_service.sh &

wait
echo "All scripts are running in the background."