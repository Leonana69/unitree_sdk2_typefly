import pyrealsense2 as rs
import numpy as np
import cv2
import gi
import time
from collections import deque
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
Gst.init(None)
HOST = "230.1.1.1"
PUBLISH_RATE = 15
FRAME_RATE = 30

# ==== CAMERA SETTINGS ====
USE_MASTER_CAMERA_SETTINGS = True  # Use first camera's auto settings for all
AUTO_CALIBRATION_FRAMES = 30  # Number of frames to let auto-exposure settle

def gst_pipeline(port, height):
    """Creates a UDP GStreamer pipeline string for RK3588"""
    return (
        f"appsrc name=appsrc is-live=true block=true format=TIME ! "
        "videoconvert ! "
        f"video/x-raw,format=NV12,width=640,height={height},framerate={PUBLISH_RATE}/1 ! "
        f"mpph264enc bps={300000 * (height // 480)} header-mode=1 ! "
        "rtph264pay ! "
        f"udpsink host={HOST} port={port} auto-multicast=true multicast-iface=wlan0 sync=false"
    )

def create_gst_app(port, width, height):
    pipeline_str = gst_pipeline(port, height)
    pipeline = Gst.parse_launch(pipeline_str)
    appsrc = pipeline.get_by_name("appsrc")
    appsrc.set_property("is-live", True)
    appsrc.set_property("format", Gst.Format.TIME)
    appsrc.set_property("caps",
        Gst.Caps.from_string(
            f"video/x-raw,format=BGR,width={width},height={height},framerate={PUBLISH_RATE}/1"
        )
    )
    pipeline.set_state(Gst.State.PLAYING)
    return pipeline, appsrc

def get_camera_serials():
    """Get serial numbers of connected D435i cameras"""
    ctx = rs.context()
    devices = ctx.query_devices()
    serials = []
    for dev in devices:
        serials.append(dev.get_info(rs.camera_info.serial_number))
    return serials

def get_camera_settings(pipeline):
    """Read current settings from a camera"""
    profile = pipeline.get_active_profile()
    device = profile.get_device()
    color_sensor = device.first_color_sensor()
    
    settings = {
        'exposure': color_sensor.get_option(rs.option.exposure),
        'gain': color_sensor.get_option(rs.option.gain),
        'white_balance': color_sensor.get_option(rs.option.white_balance),
    }
    
    # Try to get additional settings if available
    try:
        settings['brightness'] = color_sensor.get_option(rs.option.brightness)
        settings['contrast'] = color_sensor.get_option(rs.option.contrast)
        settings['saturation'] = color_sensor.get_option(rs.option.saturation)
        settings['sharpness'] = color_sensor.get_option(rs.option.sharpness)
    except:
        pass
    
    return settings

def apply_camera_settings(pipeline, settings):
    """Apply settings to a camera"""
    profile = pipeline.get_active_profile()
    device = profile.get_device()
    color_sensor = device.first_color_sensor()
    
    # Disable auto modes
    color_sensor.set_option(rs.option.enable_auto_exposure, 0)
    color_sensor.set_option(rs.option.enable_auto_white_balance, 0)
    
    # Apply main settings
    color_sensor.set_option(rs.option.exposure, settings['exposure'])
    color_sensor.set_option(rs.option.gain, settings['gain'])
    color_sensor.set_option(rs.option.white_balance, settings['white_balance'])
    
    # Apply additional settings if available
    try:
        if 'brightness' in settings:
            color_sensor.set_option(rs.option.brightness, settings['brightness'])
        if 'contrast' in settings:
            color_sensor.set_option(rs.option.contrast, settings['contrast'])
        if 'saturation' in settings:
            color_sensor.set_option(rs.option.saturation, settings['saturation'])
        if 'sharpness' in settings:
            color_sensor.set_option(rs.option.sharpness, settings['sharpness'])
    except:
        pass

def enable_auto_exposure(pipeline):
    """Enable auto-exposure and auto white balance on a camera"""
    profile = pipeline.get_active_profile()
    device = profile.get_device()
    color_sensor = device.first_color_sensor()
    
    color_sensor.set_option(rs.option.enable_auto_exposure, 1)
    color_sensor.set_option(rs.option.enable_auto_white_balance, 1)

def stream_realsense():
    # ==== Get available cameras ====
    serials = get_camera_serials()
    num_cameras = len(serials)
    
    if num_cameras == 0:
        print("❌ No RealSense cameras found!")
        return
    
    print(f"📷 Found {num_cameras} camera(s): {serials}")
    
    # Calculate output dimensions
    output_width = 640
    output_height = 480 * num_cameras
    
    # ==== Initialize RealSense pipelines ====
    pipelines = []
    for i, serial in enumerate(serials):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, FRAME_RATE)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, FRAME_RATE)
        pipeline.start(config)
        pipelines.append(pipeline)
        print(f"✓ Started camera {i+1}: {serial}")

    # ==== Synchronize camera settings ====
    if USE_MASTER_CAMERA_SETTINGS and num_cameras > 1:
        print(f"\n⚙️  Calibrating master camera (camera 1)...")
        master_pipeline = pipelines[0]
        
        # Enable auto-exposure on master camera
        enable_auto_exposure(master_pipeline)
        
        # Let auto-exposure settle
        print(f"  └─ Waiting {AUTO_CALIBRATION_FRAMES} frames for auto-exposure to stabilize...")
        for _ in range(AUTO_CALIBRATION_FRAMES):
            master_pipeline.wait_for_frames()
            time.sleep(0.01)
        
        # Read settings from master camera
        master_settings = get_camera_settings(master_pipeline)
        print(f"\n📊 Master camera settings:")
        print(f"  ├─ Exposure: {master_settings['exposure']:.1f} μs")
        print(f"  ├─ Gain: {master_settings['gain']:.1f}")
        print(f"  └─ White Balance: {master_settings['white_balance']:.0f} K")

        master_settings['white_balance'] = 6400.0
        
        # Apply master settings to all cameras (including master)
        print(f"\n🔄 Applying settings to all cameras...")
        for i, pipeline in enumerate(pipelines):
            apply_camera_settings(pipeline, master_settings)
            print(f"  ✓ Camera {i+1} synchronized")
    
    print(f"\n✓ All cameras configured and synchronized!")

    # ==== Create GStreamer senders ====
    color_pipe, color_src = create_gst_app(port=1722, width=output_width, height=output_height)
    depth_pipe, depth_src = create_gst_app(port=1723, width=output_width, height=output_height)
    print(f"✓ GStreamer pipelines ready: {output_width}x{output_height} on ports 1722 (color) and 1723 (depth)")

    align_to = rs.stream.color
    align = rs.align(align_to)
    hole_filling = rs.hole_filling_filter()

    frame_buffers = [deque(maxlen=10), deque(maxlen=10)]
    max_time_diff_ms = 40  # ~1 frame at 30fps
    frame_count = 0

    try:
        while True:
            # Collect frames from both cameras into buffers
            for i, pipeline in enumerate(pipelines):
                try:
                    frames = pipeline.poll_for_frames()
                    if frames:
                        frame_buffers[i].append(frames)
                except:
                    pass

            best_match = []
            best_time_diff = float('inf')
            if num_cameras > 1:
                for frames0 in frame_buffers[0]:
                    for frames1 in frame_buffers[1]:
                        time_diff = abs(frames0.get_timestamp() - frames1.get_timestamp())
                        if time_diff < best_time_diff:
                            best_time_diff = time_diff
                            best_match = [frames0, frames1]
            else:
                best_match.append(frame_buffers[0][-1])
                best_time_diff = 0

            if best_match and best_time_diff < max_time_diff_ms:
                frame_count += 1

                color_images = []
                depth_images = []
            
                # Get frames from all cameras
                all_frames_valid = True
                for i, frames in enumerate(best_match):
                    aligned_frames = align.process(frames)
                    color_frame = aligned_frames.get_color_frame()
                    depth_frame = aligned_frames.get_depth_frame()
                    depth_frame = hole_filling.process(depth_frame)

                    if frames in frame_buffers[i]:
                        frame_buffers[i].remove(frames)
                    
                    if not color_frame or not depth_frame:
                        all_frames_valid = False
                        break

                    # Convert to numpy arrays
                    color_images.append(np.asanyarray(color_frame.get_data()))
                    depth_images.append(np.asanyarray(depth_frame.get_data()))

                
                
                if not all_frames_valid:
                    continue
            else:
                continue

            # Stack images if multiple cameras, otherwise use single image
            if num_cameras > 1:
                color_combined = np.vstack(color_images)
                
                # Normalize and stack depth images
                depth_bgr_images = []
                for depth_image in depth_images:
                    depth_normalized = np.clip(depth_image / 25, 0, 255).astype(np.uint8)
                    depth_gray_bgr = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)
                    depth_bgr_images.append(depth_gray_bgr)
                depth_combined = np.vstack(depth_bgr_images)
            else:
                color_combined = color_images[0]
                depth_normalized = np.clip(depth_images[0] / 25, 0, 255).astype(np.uint8)
                depth_combined = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)

            # --- Push COLOR frame ---
            color_bytes = color_combined.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(color_bytes), None)
            buf.fill(0, color_bytes)
            timestamp = Gst.util_uint64_scale(GLib.get_monotonic_time(), Gst.SECOND, 1000000)
            buf.pts = buf.dts = timestamp
            buf.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, PUBLISH_RATE)
            color_src.emit("push-buffer", buf)

            # --- Push DEPTH frame ---
            depth_bytes = depth_combined.tobytes()
            buf2 = Gst.Buffer.new_allocate(None, len(depth_bytes), None)
            buf2.fill(0, depth_bytes)
            buf2.pts = buf2.dts = timestamp
            buf2.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, PUBLISH_RATE)
            depth_src.emit("push-buffer", buf2)

            # Optional: preview locally
            # cv2.imshow("Color", color_combined)
            # cv2.imshow("Depth", depth_combined)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

    finally:
        for pipeline in pipelines:
            pipeline.stop()
        color_pipe.set_state(Gst.State.NULL)
        depth_pipe.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()
        print("🛑 Stopped streaming")

if __name__ == "__main__":
    stream_realsense()