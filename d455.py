import pyrealsense2 as rs
import numpy as np
import cv2

pipe = rs.pipeline()
cfg = rs.config()

cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipe.start(cfg)

while True:
	frame = pipe.wait_for_frames()
	depth_frame = frame.get_depth_frame()
	
	depth_image = np.asanyarray(depth_frame.get_data())
	depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha = 0.03), cv2.COLORMAP_JET)
	cv2.imwrite("data/render/d455_render.png", depth_image)
	exit()
	cv2.imshow("depth", depth_image)
	
	if cv2.waitKey(1) == ord('q'):
		break
	
pipe.stop()