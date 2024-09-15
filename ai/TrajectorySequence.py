import time
import numpy as np

class TrajectorySequence:
	def __init__(self, start_yaw, start_speed, start_direction, end_yaw, end_speed, end_direction, duration):
		self.start_yaw = start_yaw
		self.start_speed = start_speed
		self.start_direction = start_direction
		self.end_yaw = end_yaw
		self.end_speed = end_speed
		self.end_direction = end_direction
		self.duration = duration
		
		self.start_time = None
		
	def StartTimer(self):
		self.start_time = time.time()
		
	def IsComplete(self):
		is_complete = False
		
		if time.time() - self.start_time > self.duration:
			is_complete = True
			
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
			return self.start_yaw, self.start_speed, self.start_direction
		
		interpolation = self.GetCompletion()
		
		if interpolation > 1.0:
			interpolation = 1.0
		
		yaw = (interpolation * self.end_yaw) + ((1 - interpolation) * self.start_yaw)
		speed = (interpolation * self.end_speed) + ((1 - interpolation) * self.start_speed)
		direction = (interpolation * self.end_direction) + ((1 - interpolation) * self.start_direction)
		
		direction_size = np.linalg.norm(direction)
		if direction_size == 0:
			direction_size = 1
			
		direction = direction / direction_size
		
		return yaw, speed, direction
		