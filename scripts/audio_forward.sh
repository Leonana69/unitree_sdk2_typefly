#!/bin/bash

while ! pgrep -f '/unitree/module/audio_hub/audiohub'; do
    sleep 1
done

gst-launch-1.0 -v \
    alsasrc device=default ! \
    audioconvert ! audioresample ! \
    opusenc ! rtpopuspay ! \
    udpsink host=230.1.1.2 port=1722 \
    auto-multicast=true multicast-iface=wlan0