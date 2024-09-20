import time
import numpy as np
import math

import ai.PerceptionCortex as PerceptionCortex
import ai.Pid as Pid

class FlightColorAlignSequence:
	def __init__(self, speed, duration, depth_camera, video_writer = None):
		self.speed = speed
		self.duration = duration
		
		self.depth_camera = depth_camera
		self.video_writer = video_writer
		self.perception_cortex = PerceptionCortex.PerceptionCortex(depth_camera, video_writer)
		
		self.vertical_pid = Pid.Pid(
			p_scale = 1,
			i_scale = 0,
			d_scale = 0
		)
		self.lateral_pid = Pid.Pid(
			p_scale = 0.0005,
			i_scale = 0,
			d_scale = 0
		)
		
		self.target_lateral = 320
		self.target_height = -5
		
		self.start_time = None
		
	def StartTimer(self):
		self.start_time = time.time()
		
		if self.video_writer is not None:
			print("<Recording D455 Video>")
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
	
	def GetTrajectory(self, data):
		if self.start_time is None:
			return None
		
		heading = data["heading"]
		position = data["position"]
		velocity = data["velocity"]
		
		pixel_position = self.perception_cortex.GetTargetPixelPosition()
		
		lateral_error = 0
		if pixel_position is not None:
			lateral_error = self.lateral_pid.ControlStep(pixel_position[0], self.target_lateral)
			
		vertical_error = self.vertical_pid.ControlStep(position[2], self.target_height, velocity[2])
		
		heading_error = np.clip(lateral_error, -math.pi, math.pi)
		
		yaw = heading + heading_error
		speed = self.speed
		direction = np.array(
			[
				math.cos(yaw),
				math.sin(yaw),
				vertical_error
			],
			np.float32
		)
		
		direction_size = np.linalg.norm(direction)
		if direction_size == 0:
			direction_size = 1
			
		direction = direction / direction_size
		
		velocity = speed * direction
		velocity[2] = -0.5#vertical_error
		
		return yaw, velocity
		