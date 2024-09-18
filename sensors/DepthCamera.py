import pyrealsense2 as rs
import numpy as np
import cv2

class DepthCamera:
	def __init__(self):
		self.resolution = np.array([480, 640])
		self.height = self.resolution[0]
		self.width = self.resolution[1]
		self.fps = 30
		
		self.data_pipe = rs.pipeline()
		self.aligner = rs.align(rs.stream_color)
		
		self.rs_config = rs.config()
		self.rs_config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
		self.rs_config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
		
		self.data_pipe.start(self.rs_config)
		
	def GetImageData(self):
		frames = pipe.wait_for_frames()
		
		aligned_frames = self.aligner.process(frames)
		
		depth_frame = aligned_frames.get_depth_frame()
		color_frame = aligned_frames.get_color_frame()
		
		depth_image = np.asanyarray(depth_frame.get_data())
		color_image = np.asanyarray(color_frame.get_data())
		
		return depth_image, color_image
	
	def GetColorImage(self):
		pass

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









