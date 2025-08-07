
# Run UV mapping calibration using frames loaded from a recorded bag file.
# The calibration expects 1280x720 frames for left infrared, depth and color streams with 4 dot target in center ROI
# Because playback device does not have calibration capabilities, a live D400 camera will need to be connected while running this script

import pyrealsense2 as rs
import time

# To activate logging uncomment the following line
# rs.log_to_console(rs.log_severity.info)

number_of_images = 25
file_name = "uv_mapping.bag" # Update to your file name
    
ctx = rs.context()
try:
    dev = ctx.query_devices()[0]
    product_line = dev.get_info(rs.camera_info.product_line)
    if product_line != "D400":
        raise TypeError
except (IndexError, TypeError):
    print('A D400 device should be connected to run this script')

infra_queue = rs.frame_queue(capacity=number_of_images, keep_frames=True)
color_queue = rs.frame_queue(capacity=number_of_images, keep_frames=True)
depth_queue = rs.frame_queue(capacity=number_of_images, keep_frames=True)

def frames_cb(frame):
    for f in frame.as_frameset():
        p = f.get_profile()
        if p.stream_type() == rs.stream.infrared:
            infra_queue.enqueue(f)
        if p.stream_type() == rs.stream.color:
            color_queue.enqueue(f)
        if p.stream_type() == rs.stream.depth:
            depth_queue.enqueue(f)
    print(f"progress: {infra_queue.size() * 2}%")

def progress_cb(progress):
    print(f"progress: {int(progress)}%")
    
# Get frames from recording
pipeline = rs.pipeline(ctx)
cfg = rs.config()
cfg.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 30)
cfg.enable_stream(rs.stream.color, 0, 1280, 720, rs.format.rgb8, 30)
cfg.enable_stream(rs.stream.depth, 0, 1280, 720, rs.format.z16, 30)
cfg.enable_device_from_file(file_name, repeat_playback=False)
pipeline.start(cfg, frames_cb)

start_time = time.time()
while infra_queue.size() < number_of_images and color_queue.size() < number_of_images and depth_queue.size() < number_of_images:
    time.sleep(0.5)
    if 10 < (time.time() - start_time):
        print(f"Failed to capture {number_of_images} frames")

pipeline.stop()

# Start the calibration
px_py_only = False 
adev = dev.as_auto_calibrated_device()
table, health = adev.run_uv_map_calibration(infra_queue, color_queue, depth_queue, px_py_only, progress_cb)

# Check results
print("Calibration returned, health:", health)
