import time
import numpy as np

import ai.PerceptionCortex as PerceptionCortex
import ai.Pid as Pid

class FlightColorAlignSequence:
	def __init__(self, speed, duration, depth_camera, video_writer = None):
		self.speed = speed
		self.duration = duration
		
		self.depth_camera = depth_camera
		self.video_writer = video_writer
		self.perception_cortex = PerceptionCortex.PerceptionCortex(depth_camera, video_writer)
		self.pid = Pid.Pid(
			p_scale = 0.1,
			i_scale = 0,
			d_scale = 0
		)
		
		self.target_state = np.array([320, 0])
		
		self.start_time = None
		
	def StartTimer(self):
		self.start_time = time.time()
		
		if self.video_writer is not None:
			self.video_writer.Start()
		
	def IsComplete(self):
		is_complete = False
		
		if time.time() - self.start_time > self.duration:
			is_complete = True
			
			if self.video_writer is not None:
				self.video_writer.Release()
			
		return is_complete
		
	def GetCompletion(self):
		time_passed = self.GetTimerValue()
		
		return time_passed / self.duration
	
	def GetTimerValue(self):
		time_passed = time.time() - self.start_time
		
		return time_passed
	
	def GetTimerValueStr(self):
		time_passed = self.GetTimerValue()
		
		return "{:.2f}".format(time_passed)
	
	def GetTrajectory(self):
		if self.start_time is None:
			return None
		
		self.perception_cortex.GetTargetPixelPosition()
		
		yaw = 0.0
		speed = 0.0
		direction = np.ones(3, np.float32)
		
		direction_size = np.linalg.norm(direction)
		if direction_size == 0:
			direction_size = 1
			
		direction = direction / direction_size
		
		return yaw, speed, direction
		