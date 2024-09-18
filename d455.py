import pyrealsense2 as rs
import numpy as np
import cv2

pipe = rs.pipeline()
aligner = rs.align(rs.stream.color)

rs_config = rs.config()
rs_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipe.start(rs_config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

max_depth = 5

while True:
	camera_data = pipe.wait_for_frames()
	camera_data = aligner.process(camera_data)
	
	depth_frame = camera_data.get_depth_frame()
	color_frame = camera_data.get_color_frame()
	
	depth_image = np.asanyarray(depth_frame.get_data())
	depth_image = depth_image.astype(np.float32)
	depth_image = depth_image * depth_scale
	depth_image = depth_image / max_depth
	depth_image = np.clip(depth_image, 0, 1)
	depth_image = depth_image * 255
	depth_image = depth_image.astype(np.uint8)
	
	color_image = np.asanyarray(color_frame.get_data())
	
	merged_image = np.concatenate((depth_image, color_image), dim = 0)
	
	cv2.imwrite("data/render/d455_render.png", merged_image)
	exit()
	
	cv2.imshow("depth", depth_image)
	
	if cv2.waitKey(1) == ord('q'):
		break
	
pipe.stop()