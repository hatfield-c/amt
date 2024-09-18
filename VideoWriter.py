import numpy as np
import cv2

class VideoWriter:
	def __init__(self):
		self.video_path = "data/render/d455_render.mp4"
		self.video_object = cv2.VideoWriter(
			self.video_path,
			cv2.VideoWriter_fourcc('m', 'p', '4', 'v'),
			10,
			(640, int(480 * 2))
		)
		
	def WritePair(self, depth_image, color_image):
		depth_image = depth_image.reshape(480, 640, 1)
		depth_image = np.tile(depth_image, (1, 3))

		merged_image = np.concatenate((depth_image, color_image), axis = 0)
		
		self.Write(merged_image)
		
	def Write(self, img):
		self.video_object.write(img)
		
	def Release(self):
		self.video_object.Release()