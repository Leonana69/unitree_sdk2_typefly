gst-launch-1.0 v4l2src device=/dev/video6 ! \
    video/x-raw,format=YUY2,width=1280,height=720,framerate=15/1 ! \
    videoconvert ! \
    mpph264enc bps=3000000 header-mode=1 ! \
    rtph264pay ! \
    udpsink host=230.1.1.1 port=1722 auto-multicast=true multicast-iface=wlan0 sync=false