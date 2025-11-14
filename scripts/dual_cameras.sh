#!/bin/bash

ROBOT_ID=${ROBOT_ID:-1}
MULTICAST_IP="230.1.1.${ROBOT_ID}"
GSTREAMER_RGB_PORT=${GSTREAMER_RGB_PORT:-1722}

gst-launch-1.0 v4l2src device=/dev/video2 do-timestamp=true ! \
    image/jpeg,width=1600,height=600,framerate=20/1 ! \
    rtpjpegpay quality=80 timestamp-offset=0 ! \
    udpsink host=${MULTICAST_IP} port=${GSTREAMER_RGB_PORT} auto-multicast=true multicast-iface=wlan0 sync=false