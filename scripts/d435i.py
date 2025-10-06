import pyrealsense2 as rs
import numpy as np
import cv2
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
Gst.init(None)
HOST = "230.1.1.1"

"""
gst-launch-1.0 \
  udpsrc address=230.1.1.1 port=1722 multicast-iface=wlo1 \
  ! queue !  application/x-rtp, media=video, encoding-name=H264 \
  ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! fpsdisplaysink
"""

def gst_pipeline(port):
    """Creates a UDP GStreamer pipeline string for RK3588"""
    return (
        f"appsrc name=appsrc is-live=true block=true format=TIME ! "
        "videoconvert ! "
        "video/x-raw,format=NV12,width=640,height=480,framerate=30/1 ! "
        "mpph264enc bps=300000 header-mode=1 ! "
        "rtph264pay ! "
        f"udpsink host={HOST} port={port} auto-multicast=true multicast-iface=wlan0 sync=false"
    )

def create_gst_app(port, width, height):
    pipeline_str = gst_pipeline(port)
    pipeline = Gst.parse_launch(pipeline_str)
    appsrc = pipeline.get_by_name("appsrc")
    appsrc.set_property("is-live", True)
    appsrc.set_property("format", Gst.Format.TIME)
    appsrc.set_property("caps",
        Gst.Caps.from_string(
            f"video/x-raw,format=BGR,width={width},height={height},framerate=30/1"
        )
    )
    pipeline.set_state(Gst.State.PLAYING)
    return pipeline, appsrc

def stream_realsense():
    # ==== Initialize RealSense ====
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    print("Started RealSense streaming...")

    # ==== Create GStreamer senders ====
    color_pipe, color_src = create_gst_app(port=1722, width=640, height=480)
    depth_pipe, depth_src = create_gst_app(port=1723, width=640, height=480)
    print("GStreamer pipelines ready on ports 1722 (color) and 1723 (depth)")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # --- Encode depth as RG (16-bit split) ---
            depth_high = (depth_image >> 8).astype(np.uint8)
            depth_low = (depth_image & 0xFF).astype(np.uint8)
            depth_rgb = np.stack((depth_high, depth_low, np.zeros_like(depth_high)), axis=-1)

            # --- Push COLOR frame ---
            color_bytes = color_image.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(color_bytes), None)
            buf.fill(0, color_bytes)
            timestamp = Gst.util_uint64_scale(GLib.get_monotonic_time(), Gst.SECOND, 1000000)
            buf.pts = buf.dts = timestamp
            buf.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, 30)
            color_src.emit("push-buffer", buf)

            # --- Push DEPTH frame ---
            depth_bytes = depth_rgb.tobytes()
            buf2 = Gst.Buffer.new_allocate(None, len(depth_bytes), None)
            buf2.fill(0, depth_bytes)
            buf2.pts = buf2.dts = timestamp
            buf2.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, 30)
            depth_src.emit("push-buffer", buf2)

    finally:
        pipeline.stop()
        color_pipe.set_state(Gst.State.NULL)
        depth_pipe.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()
        print("🛑 Stopped streaming")

if __name__ == "__main__":
    stream_realsense()