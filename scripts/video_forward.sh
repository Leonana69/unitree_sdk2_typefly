#!/bin/bash

while ! pgrep -f '/unitree/module/video_hub/videohub'; do
    sleep 1
done

gst-launch-1.0 v4l2src device=/dev/video2 ! \
    image/jpeg,width=2560,height=720,framerate=30/1 ! \
    jpegdec ! \
    mpph264enc bps=3000000 header-mode=1 ! \
    rtph264pay config-interval=-1 ! \
    udpsink host=230.1.1.1 port=1722 auto-multicast=true multicast-iface=wlan0 sync=false

# gst-launch-1.0 -v \
#   udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 \
#   ! application/x-rtp, media=video, encoding-name=H264 \
#   ! queue \
#   ! udpsink host=230.1.1.2 port=1720 auto-multicast=true multicast-iface=wlan0

# python3 /root/unitree_sdk2_typefly/scripts/d435i.py &

# gst-launch-1.0 -v \
#   udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 \
#   ! udpsink host=230.1.1.2 port=1721 auto-multicast=true multicast-iface=wlan0