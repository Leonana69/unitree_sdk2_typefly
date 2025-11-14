#!/bin/bash

ROBOT_ID=${ROBOT_ID:-1}
MULTICAST_IP="230.1.1.${ROBOT_ID}"
GSTREAMER_RGB_PORT=${GSTREAMER_RGB_PORT:-1722}
GSTREAMER_DEPTH_PORT=${GSTREAMER_DEPTH_PORT:-1723}

while ! pgrep -f '/unitree/module/video_hub/videohub'; do
    sleep 1
done

#### Dual fisheye camera forwarding
# gst-launch-1.0 v4l2src device=/dev/video2 do-timestamp=true ! \
#    image/jpeg,width=1600,height=600,framerate=30/1 ! \
#    rtpjpegpay quality=80 timestamp-offset=0 ! \
#    udpsink host=${MULTICAST_IP} port=${GSTREAMER_RGB_PORT} auto-multicast=true multicast-iface=wlan0 sync=false

#### Go2 native camera forwarding
# gst-launch-1.0 -v \
#   udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 \
#   ! udpsink host=${MULTICAST_IP} port=${GSTREAMER_RGB_PORT} auto-multicast=true multicast-iface=wlan0

# gst-launch-1.0 -v \
#   udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 \
#   ! application/x-rtp, media=video, encoding-name=H264 \
#   ! queue \
#   ! udpsink host=${MULTICAST_IP} port=${GSTREAMER_RGB_PORT} auto-multicast=true multicast-iface=wlan0

#### Depth Camera D435i Streaming
python3 /root/unitree_sdk2_typefly/scripts/d435i.py