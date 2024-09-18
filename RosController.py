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

import ai.TrajectorySequence as TrajectorySequence
import ai.ConstantSequence as ConstantSequence
import ai.FlightColorAlignSequence as FlightColorAlignSequence

import VideoWriter
import sensors.DepthCamera as DepthCamera

class RosController(Node):

	def __init__(self, is_write_video = False):
		super().__init__('minimal_publisher')
		
		self.forward_heading = 0
		self.backward_heading = math.pi
		
		self.up_direction = np.array([0, 0, -1], np.float32)
		self.forward_direction = np.array([1, 0, 0], dtype = np.float32)
		self.backward_direction = -self.forward_direction
		
		self.takeoff_speed = 10.0
		self.flight_speed = 10.0
		
		self.takeoff_duration = 5
		self.forward_duration = 3
		self.backward_duration = 16
		
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
		self.heading = 0
		self.heading_smooth = 0
		
		self.trajectory_sequences = {}
		self.sequence_states = []
		self.sequence_state_index = 0
		self.system_state = "warmup"
		self.cycles = 0
		
		self.depth_camera = DepthCamera.DepthCamera()
		self.video_writer = None
		if is_write_video:
			self.video_writer = VideoWriter.VideoWriter()
		
		self.ResetOrientation()
		
		update_period = 0.02
		self.update_timer = self.create_timer(update_period, self.Update)
		
	def ResetOrientation(self):
		self.forward_heading = self.heading_smooth
		self.backward_heading = self.forward_heading - math.pi
		if self.forward_heading < 0:
			self.backward_heading = self.forward_heading + math.pi
			
		self.forward_direction = np.array([math.cos(self.forward_heading), math.sin(self.forward_heading), 0], dtype = np.float32)
		self.backward_direction = -self.forward_direction
		
		self.forward_direction[2] = -1
		self.backward_direction[2] = -1

		self.trajectory_sequences = {
			"0_takeoff": TrajectorySequence.TrajectorySequence(
				start_yaw = self.forward_heading,
				start_speed = self.takeoff_speed,
				start_direction = self.up_direction,
				end_yaw = self.forward_heading,
				end_speed = self.flight_speed,
				end_direction = self.forward_direction,
				duration = self.takeoff_duration
			),
			"1_forward": FlightColorAlignSequence.FlightColorAlignSequence(
				yaw = self.forward_heading,
				speed = self.flight_speed,
				direction = self.forward_direction,
				duration = self.forward_duration,
				depth_camera = self.depth_camera,
				video_writer = self.video_writer
			),
			"2_backward": ConstantSequence.ConstantSequence(
				yaw = self.backward_heading,
				speed = self.flight_speed,
				direction = self.backward_direction,
				duration = self.backward_duration
			),
			"4_brakes": ConstantSequence.ConstantSequence(
				yaw = self.backward_heading,
				speed = self.flight_speed * 0,
				direction = self.backward_direction,
				duration = self.backward_duration
			),
		}
		
		self.sequence_states = sorted(list(self.trajectory_sequences.keys()))
		self.sequence_state_index = 0
		self.system_state = "warmup"
		self.cycles = 0
		
	def Update(self):
		self.PrepareToCommand()
		
		if not self.is_armed or not self.is_offboard:
			return
		
		self.TakeAction()
		
		self.cycles += 1

	def TakeAction(self):
		velocity = None
		heading = None
		
		if self.system_state == "warmup":
			print(self.system_state, self.cycles)
			
			self.WarmUp()
			
			if self.cycles > 200:
				self.system_state = "hot"
				
				sequence_state = self.sequence_states[self.sequence_state_index]
				trajectory_sequence = self.trajectory_sequences[sequence_state]
				trajectory_sequence.StartTimer()
				
		elif self.system_state == "hot":
			
			sequence_state = self.sequence_states[self.sequence_state_index]
			trajectory_sequence = self.trajectory_sequences[sequence_state]
			
			heading, speed, direction = trajectory_sequence.GetTrajectory()
			velocity = speed * direction
			
			print_header = "[" + str(sequence_state) + " : " + trajectory_sequence.GetTimerValueStr() + "]"
			print(print_header, heading, speed, direction, self.velocity)
			
			if trajectory_sequence.IsComplete():
				if sequence_state == self.sequence_states[-1]:
					self.system_state = "descend"
					
				self.sequence_state_index = min(
					self.sequence_state_index + 1, 
					len(self.sequence_states) - 1
				)
				
				if sequence_state != self.sequence_states[-1]:
					next_sequence_state = self.sequence_states[self.sequence_state_index]
					next_sequence = self.trajectory_sequences[next_sequence_state]
					next_sequence.StartTimer()
				
		elif self.system_state == "descend":
			print(self.system_state, self.velocity)
			
			velocity = -self.up_direction / 2
			heading = self.backward_heading
		
		if velocity is not None and heading is not None:
			self.SetTrajectory(velocity, heading)

	def PrepareToCommand(self):
		msg = OffboardControlMode()
		
		msg.timestamp = int(Clock().now().nanoseconds / 1000)
		
		if self.system_state == "warmup":
			msg.direct_actuator = True
		elif self.system_state in ["hot", "descend"]:
			msg.velocity = True
		
		self.publisher_offboard_mode.publish(msg)

	def SetTrajectory(self, velocity, heading):
		msg = TrajectorySetpoint()
		msg.timestamp = int(Clock().now().nanoseconds / 1000)
		msg.position[0] = np.nan
		msg.position[1] = np.nan
		msg.position[2] = np.nan
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
		
		self.heading = msg.heading
		self.heading_smooth = ((2 / 3) * self.heading_smooth) + ((1 / 3) * msg.heading)

	def ControlModeCallback(self, msg):
		msg_is_armed = msg.flag_armed
		msg_is_offboard = msg.flag_control_offboard_enabled
		
		if msg_is_offboard and not self.is_offboard:
			print("<Offboard control activated!>")
			self.ResetOrientation()
			
		self.is_armed = msg_is_armed
		self.is_offboard = msg_is_offboard
		