import pyrealsense2 as rs
import numpy as np
import cv2

class DepthCamera:
	def __init__(self, resolution, fps):
		self.resolution = resolution
		self.height = resolution[0]
		self.width = resolution[1]
		self.fps = fps
		
		self.data_pipe = rs.pipeline()
		
		self.rs_config = rs.config()
		self.rs_config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, fps)
		
		self.data_pipe.start(self.rs_config)
		
		#self.data_pipe = rs.pipeline()
		#self.data_pipe.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
		#enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
		

pipe = rs.pipeline()
cfg = rs.config()

cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)



while True:
	frame = pipe.wait_for_frames()
	depth_frame = frame.get_depth_frame()
	
	depth_image = np.asanyarray(depth_frame.get_data())
	
	cv2.imshow("depth", depth_image)
	
	if cv2.waitKey(1) == ord('q'):
		break
	
pipe.stop()