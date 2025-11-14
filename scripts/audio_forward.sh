#!/bin/bash

while ! pgrep -f '/unitree/module/audio_hub/audiohub'; do
    sleep 1
done

ROBOT_ID=${ROBOT_ID:-1}
MULTICAST_IP="230.1.1.${ROBOT_ID}"
GSTREAMER_AUDIO_PORT=${GSTREAMER_AUDIO_PORT:-1724}

gst-launch-1.0 -v \
    alsasrc device=default ! \
    audioconvert ! audioresample ! \
    opusenc ! rtpopuspay ! \
    udpsink host=${MULTICAST_IP} port=${GSTREAMER_AUDIO_PORT} \
    auto-multicast=true multicast-iface=wlan0