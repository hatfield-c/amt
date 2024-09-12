import time
import math

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import ActuatorServos
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint 
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition

class RosController(Node):

	def __init__(self):
		super().__init__('minimal_publisher')
		
		self.takeoff_duration = 3
		self.flight_duration = 5
		self.fixed_heading = 1.55
		
		self.takeoff_speed = 3
		self.flight_speed = 3
		
		self.flight_velocity = np.array([math.cos(self.fixed_heading), math.sin(self.fixed_heading), 0], dtype = np.float32) * self.flight_speed
		
		qos_profile = QoSProfile(
			reliability = QoSReliabilityPolicy.BEST_EFFORT,
			durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
			history = QoSHistoryPolicy.KEEP_LAST,
			depth = 1
		)
		
		self.position_sub = self.create_subscription(
			VehicleLocalPosition,
			'/fmu/out/vehicle_local_position',
			self.PositionCallback,
			qos_profile
		)
		
		self.control_mode_sub = self.create_subscription(
			VehicleControlMode,
			'/fmu/out/vehicle_control_mode',
			self.ControlModeCallback,
			qos_profile
		)
		
		self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
		self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)
		self.trajectory_publisher = self.create_publisher(TrajectorySetpoint , "/fmu/in/trajectory_setpoint", qos_profile)
		self.motor_publisher = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", qos_profile)
		self.servo_publisher = self.create_publisher(ActuatorServos, "/fmu/in/actuator_servos", qos_profile)
		
		self.is_armed = False
		self.is_offboard = False
		
		self.velocity = np.zeros(3, dtype = np.float32)
		
		update_period = 0.02
		self.update_timer = self.create_timer(update_period, self.Update)
		
		self.cycles = 0
		self.takeoff_start_time = time.time()
		self.flight_start_time = time.time()
		self.current_state = "warmup"
		
		print("<uORB RosController Initialzed!>")
		
	def Update(self):
		self.PrepareToCommand()
		
		if not self.is_armed or not self.is_offboard:
			return
		
		self.TakeAction()
		
		self.cycles += 1

	def TakeAction(self):
		if self.current_state == "warmup":
			print(self.current_state, self.cycles)
			
			self.WarmUp()
			
			if self.cycles > 200:
				self.current_state = "takeoff"
				self.takeoff_start_time = time.time()
				
		elif self.current_state == "takeoff":
			print(self.current_state, self.velocity)
			
			self.SetTrajectory(np.array([0, 0, -self.takeoff_speed], dtype = np.float32), self.fixed_heading)
			
			if time.time() - self.takeoff_start_time > self.takeoff_duration:
				#self.current_state = "flight"
				#self.flight_start_time = time.time()
				self.current_state = "idle"
				
		elif self.current_state == "flight":
			print(self.current_state, self.velocity)
			
			self.SetTrajectory(self.flight_velocity, self.fixed_heading)
			
			if time.time() - self.flight_start_time > self.flight_duration:
				self.current_state = "return"
				self.flight_start_time = time.time()
		
		elif self.current_state == "return":
			print(self.current_state, self.velocity)
			
			reverse_velocity = -self.flight_velocity
			reverse_heading = self.fixed_heading - math.pi
			if self.fixed_heading < 0:
				reverse_heading = self.fixed_heading + math.pi
			
			self.SetTrajectory(reverse_velocity, reverse_heading)
			
			if time.time() - self.flight_start_time > self.flight_duration:
				self.current_state = "idle"
		
		elif self.current_state == "idle":
			print(self.current_state, self.velocity)
			
			self.SetTrajectory(np.zeros(3, dtype = np.float32), self.fixed_heading)

	def PrepareToCommand(self):
		msg = OffboardControlMode()
		
		msg.timestamp = int(Clock().now().nanoseconds / 1000)
		
		if self.current_state == "warmup":
			msg.direct_actuator = True
			
		if self.current_state in ["takeoff", "flight", "return", "idle"]:
			msg.velocity = True
		
		self.publisher_offboard_mode.publish(msg)

	def SetTrajectory(self, velocity, heading):
		msg = TrajectorySetpoint()
		msg.timestamp = int(Clock().now().nanoseconds / 1000)
		msg.velocity = velocity
		msg.yaw = heading
		
		self.trajectory_publisher.publish(msg)

	def SetDropper(self, is_closed):
		servo_signal = -1.0
		if is_closed:
			servo_signal = 1.0
		
		msg = VehicleCommand()

		msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR 
		msg.param1 = servo_signal
		
		msg.timestamp = int(Clock().now().nanoseconds / 1000)
		msg.target_system = 1
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		
		self.vehicle_command_publisher.publish(msg)

	def WarmUp(self):
		warmup_throttle = 0.2
		
		msg = ActuatorMotors()		
		
		msg.timestamp = int(Clock().now().nanoseconds / 1000)
		msg.control[0] = warmup_throttle
		msg.control[1] = warmup_throttle
		msg.control[2] = warmup_throttle
		msg.control[3] = warmup_throttle
		
		self.motor_publisher.publish(msg)

	def PositionCallback(self, msg):
		self.velocity[0] = msg.vx
		self.velocity[1] = msg.vy
		self.velocity[2] = -msg.vz

	def ControlModeCallback(self, msg):
		msg_is_armed = msg.flag_armed
		msg_is_offboard = msg.flag_control_offboard_enabled
		
		if msg_is_offboard and not self.is_offboard:
			self.current_state = "warmup"
			self.cycles = 0
			
		self.is_armed = msg_is_armed
		self.is_offboard = msg_is_offboard