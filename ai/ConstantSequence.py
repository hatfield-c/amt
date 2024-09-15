import time
import numpy as np

class ConstantSequence:
	def __init__(self, yaw, speed, direction, duration):
		self.yaw = yaw
		self.speed = speed
		self.direction = direction
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
		
		yaw = self.yaw
		speed = self.speed
		direction = self.direction
		
		
		
		direction_size = np.linalg.norm(direction)
		if direction_size == 0:
			direction_size = 1
			
		direction = direction / direction_size
		
		return yaw, speed, direction
		