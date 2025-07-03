#!/bin/bash

/root/scripts/video_forward.sh & /root/scripts/control_interface.sh & /root/scripts/audio_forward.sh &

wait
echo "All scripts are running in the background."