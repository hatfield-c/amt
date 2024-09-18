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
		self.aligner = rs.align(rs.stream.color)
		
		self.rs_config = rs.config()
		self.rs_config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
		self.rs_config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
		
		self.profile = self.data_pipe.start(self.rs_config)

		self.depth_sensor = self.profile.get_device().first_depth_sensor()
		self.depth_scale = self.depth_sensor.get_depth_scale()

		self.max_depth = 20
		
	def GetImageData(self):
		camera_data = self.data_pipe.wait_for_frames()
		camera_data = self.aligner.process(camera_data)
		
		depth_frame = camera_data.get_depth_frame()
		color_frame = camera_data.get_color_frame()
		
		depth_image = np.asanyarray(depth_frame.get_data())
		depth_image = depth_image.astype(np.float32)
		depth_image = depth_image * self.depth_scale
		depth_image = depth_image / self.max_depth
		depth_image = np.clip(depth_image, 0, 1)
		depth_image = depth_image * 255
		depth_image = depth_image.astype(np.uint8)
		
		color_image = np.asanyarray(color_frame.get_data())
		
		return depth_image, color_image
	
	def SaveImageData(self, depth_image, color_image, index = 0):
		depth_image = depth_image.reshape(480, 640, 1)
		depth_image = np.tile(depth_image, (1, 3))

		merged_image = np.concatenate((depth_image, color_image), axis = 0)
	
		index_str = str(index).zfill(6)
		file_name = "data/render/d455_render_" + index_str +".bmp"
		cv2.imwrite(file_name, merged_image)
