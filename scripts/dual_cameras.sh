gst-launch-1.0 v4l2src device=/dev/video2 do-timestamp=true ! \
    image/jpeg,width=1600,height=600,framerate=20/1 ! \
    rtpjpegpay quality=80 timestamp-offset=0 ! \
    udpsink host=230.1.1.1 port=1722 auto-multicast=true multicast-iface=wlan0 sync=false