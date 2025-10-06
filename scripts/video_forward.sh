#!/bin/bash

while ! pgrep -f '/unitree/module/video_hub/videohub'; do
    sleep 1
done

# gst-launch-1.0 -v \
#   udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 \
#   ! application/x-rtp, media=video, encoding-name=H264 \
#   ! queue \
#   ! udpsink host=230.1.1.2 port=1720 auto-multicast=true multicast-iface=wlan0

python3 /root/unitree_sdk2_typefly/scripts/d435i.py &

# gst-launch-1.0 -v \
#   udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 \
#   ! udpsink host=230.1.1.2 port=1721 auto-multicast=true multicast-iface=wlan0