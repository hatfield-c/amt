import numpy as np
import math
import cv2
import torch

class PerceptionCortex:
	def __init__(self, depth_camera, video_writer = None):
		self.depth_camera = depth_camera
		self.video_writer = video_writer
		
		voting_rounds = 7
		median_size = 5
		dilate_size = -1
		chroma_center = np.array([150, 205, 145], np.float32)
		chroma_width = np.array([67, 24, 31], np.float32)
		
		self.voting_rounds = voting_rounds
		self.median_size = median_size
		self.dilate_size = dilate_size

		self.chroma_center = chroma_center
		self.chroma_width = chroma_width

		self.chroma_lower = self.chroma_center - self.chroma_width
		self.chroma_upper = self.chroma_center + self.chroma_width

		self.chroma_lower = np.clip(self.chroma_lower, 0, 255)
		self.chroma_upper = np.clip(self.chroma_upper, 0, 255)

		if self.median_size % 2 == 0:
			self.median_size += 1

		if self.dilate_size % 2 == 0:
			self.dilate_size += 1

		self.depth_dilate_kernel = None
		if self.dilate_size > 2:
			self.depth_dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (dilate_size, dilate_size))

		self.img_padder = torch.nn.ReplicationPad2d(1).cuda()

	def GetTargetPixelPosition(self):
	
		depth_image, color_image = self.depth_camera.GetImageData()
		
		binary_frame = self.BlobbingFilter(color_image)
		
		
		if self.video_writer is not None:
			self.video_writer.WritePair(depth_image, color_image)
			
		contours = cv2.findContours(binary_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

		camera_position = self.GetContourPosition(contours)

		cv2.line(color_image, (camera_position[0], 0), (camera_position[0], 479), (0, 255, 0), 2)
		cv2.line(color_image, (0, camera_position[1]), (639, camera_position[1]), (0, 255, 0), 2)

		cv2.imshow("render", color_image)
		cv2.waitKey(1)

		return camera_position

	def GetContourPosition(self, contours):
		biggest_area = -1
		position = None
		
		for i in range(len(contours)):
			c = contours[i]
			x, y, w, h = cv2.boundingRect(c)

			area = w * h
			position_candidate = (x + (w // 2), y + (h // 2))

			if biggest_area < area:
				biggest_area = area
				position = position_candidate

		return position

	def BlobbingFilter(self, rgb_frame):
		lab_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2Lab)
		
		binary_frame = cv2.inRange(lab_frame, self.chroma_lower, self.chroma_upper)
		binary_frame = self.VotePool(binary_frame)
		binary_frame = binary_frame.astype(np.uint8)

		if self.median_size > 2:
			binary_frame = cv2.medianBlur(binary_frame, self.median_size)

		if self.dilate_size > 2:
			binary_frame = cv2.morphologyEx(binary_frame, cv2.MORPH_DILATE, self.depth_dilate_kernel)

		return binary_frame

	def VotePool(self, img):
		img = torch.FloatTensor(img).cuda()
		img = img.reshape(1, 1, img.shape[0], img.shape[1])

		for i in range(self.voting_rounds):
			img_pad = self.img_padder(img)

			img_local = [
				torch.roll(img_pad[0], 1, dims = 1),
				torch.roll(img_pad[0], -1, dims = 1),
				torch.roll(img_pad[0], 1, dims = 2),
				torch.roll(img_pad[0], -1, dims = 2),
			]

			img_local = torch.stack(img_local)
			max_local = torch.max(img_local, dim = 0, keepdim = True).values

			diffs = img_local - img_pad
			up_diffs = torch.clamp(diffs, 0, 1)
			votes = torch.sum(up_diffs, dim = 0, keepdim = True)
			votes = votes - 1
			vote_mask = torch.clamp(votes, 0, 1)

			filled_img = (vote_mask * max_local) + ((1 - vote_mask) * img_pad)

			img = filled_img[[0], :, 1:-1, 1:-1]

		img = img[0, 0]
		img = img.cpu().numpy()

		return img
