import pyrealsense2 as rs
import numpy as np
import cv2
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
Gst.init(None)
HOST = "230.1.1.1"
FRAME_RATE = 30

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
        f"video/x-raw,format=NV12,width=640,height=480,framerate={FRAME_RATE}/1 ! "
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
            f"video/x-raw,format=BGR,width={width},height={height},framerate={FRAME_RATE}/1"
        )
    )
    pipeline.set_state(Gst.State.PLAYING)
    return pipeline, appsrc

def stream_realsense():
    # ==== Initialize RealSense ====
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, FRAME_RATE)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, FRAME_RATE)
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

            # Normalize depth for visualization
            depth_vis = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
            )
            # cap 6375 mm ~ 25 * 255
            depth_normalized = np.clip(depth_image / 25, 0, 255).astype(np.uint8)
            depth_gray_bgr = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)

            # --- Push COLOR frame ---
            color_bytes = color_image.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(color_bytes), None)
            buf.fill(0, color_bytes)
            timestamp = Gst.util_uint64_scale(GLib.get_monotonic_time(), Gst.SECOND, 1000000)
            buf.pts = buf.dts = timestamp
            buf.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, FRAME_RATE)
            color_src.emit("push-buffer", buf)

            # --- Push DEPTH frame ---
            depth_bytes = depth_gray_bgr.tobytes()
            buf2 = Gst.Buffer.new_allocate(None, len(depth_bytes), None)
            buf2.fill(0, depth_bytes)
            buf2.pts = buf2.dts = timestamp
            buf2.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, FRAME_RATE)
            depth_src.emit("push-buffer", buf2)

            # Optional: preview locally
            # cv2.imshow("Color", color_image)
            # cv2.imshow("Depth", depth_vis)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

    finally:
        pipeline.stop()
        color_pipe.set_state(Gst.State.NULL)
        depth_pipe.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()
        print("🛑 Stopped streaming")

if __name__ == "__main__":
    stream_realsense()