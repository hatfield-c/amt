import time
import torch
import math

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import ManualControlSetpoint
from px4_msgs.msg import GotoSetpoint
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import ActuatorServos
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleCommandAck
from px4_msgs.msg import VehicleThrustSetpoint

class RosController(Node):

	def __init__(self):
		super().__init__('minimal_publisher')
		qos_profile = QoSProfile(
			reliability = QoSReliabilityPolicy.BEST_EFFORT,
			durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
			history = QoSHistoryPolicy.KEEP_LAST,
			depth = 1
		)

		self.attitude_sub = self.create_subscription(
			VehicleAttitude,
			'/fmu/out/vehicle_attitude',
			self.AttitudeCallback,
			qos_profile
		)
		
		self.position_sub = self.create_subscription(
			VehicleLocalPosition,
			'/fmu/out/vehicle_local_position',
			self.PositionCallback,
			qos_profile
		)
		
		self.command_ack_sub = self.create_subscription(
			VehicleCommandAck,
			'/fmu/out/vehicle_command_ack',
			self.CommandAcknowledge,
			qos_profile
		)
		
		self.control_mode_sub = self.create_subscription(
			VehicleControlMode,
			'/fmu/out/vehicle_control_mode',
			self.ControlModeCallback,
			qos_profile
		)
		
		self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
		
		self.setpoint_publisher = self.create_publisher(GotoSetpoint, "/fmu/in/goto_setpoint", qos_profile)
		self.rc_spoofer_publisher = self.create_publisher(ManualControlSetpoint, "/fmu/in/manual_control_input", qos_profile)
		self.motor_publisher = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", qos_profile)
		self.servo_publisher = self.create_publisher(ActuatorServos, "/fmu/in/actuator_servos", qos_profile)
		self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)
		
		self.quaternion = None
		self.heading = None
		self.is_armed = False
		self.position = torch.zeros((1, 3)).cuda()
		self.velocity = torch.zeros((1, 3)).cuda()

		timer_period = 0.02
		self.timer = self.create_timer(timer_period, self.Update)
		
		self.cycles = 0
		self.max_cycles = 100
		
	def Update(self):
		
		self.PrepareToCommand()
		
		print("[Armed]:", self.is_armed)
		
		if not self.is_armed:
			return
		
		self.cycles += 1
		
		#self.SetDropperPosition(0.35)
		
		t_signal =  (math.sin(self.cycles * (math.pi / 200)) + 1) / 2
		
		self.publish_motor([t_signal, t_signal, t_signal, t_signal])
		#self.SetDropperPosition(t_signal)
		#self.SetManualRcInput(t_signal)
		#self.SetPoint(np.array([0, 0, -10], dtype = np.float32))
		
		
		#print(self.position, self.velocity, self.heading)

	def SetDropperPosition(self, position_signal):
		msg = VehicleCommand()

		msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR 
		msg.param1 = position_signal
		msg.param2 = position_signal
		msg.param3 = position_signal
		msg.param4 = position_signal
		msg.param5 = position_signal
		msg.param6 = position_signal
		msg.param7 = 0.0
		
		msg.timestamp = int(Clock().now().nanoseconds / 1000)
		msg.target_system = 1
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		
		self.vehicle_command_publisher.publish(msg)
		
	def SetManualRcInput(self, position_signal):
		msg = ManualControlSetpoint()
		msg.timestamp = int(Clock().now().nanoseconds / 1000)
		msg.roll = position_signal
		msg.pitch = position_signal
		msg.yaw = position_signal
		msg.throttle = position_signal
		
		msg.aux1 = position_signal
		msg.aux2 = position_signal
		msg.aux3 = position_signal
		msg.aux4 = position_signal
		msg.aux5 = position_signal
		msg.aux6 = position_signal
		
		self.rc_spoofer_publisher.publish(msg)

	def SetPoint(self, position):
		msg = GotoSetpoint()
		msg.timestamp = int(Clock().now().nanoseconds / 1000)
		msg.position = position
		msg.heading = 0
		
		self.setpoint_publisher.publish(msg)

	def PrepareToCommand(self):
		msg = OffboardControlMode()
		
		msg.timestamp = int(Clock().now().nanoseconds / 1000)
		msg.position = False
		msg.velocity = False
		msg.acceleration = False
		msg.attitude = False
		msg.body_rate = False
		msg.direct_actuator = True
		
		self.publisher_offboard_mode.publish(msg)

	def SetArmed(self, arm_value):
		msg = VehicleCommand()
		
		msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
		msg.param1 = arm_value
		
		msg.target_system = 1
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = int(Clock().now().nanoseconds / 1000)
		
		self.vehicle_command_publisher.publish(msg)
		
		if arm_value == 1:
			
			msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
			msg.param1 = 1.0
			msg.param2 = 6.0
			msg.timestamp = int(Clock().now().nanoseconds / 1000)
			
			self.vehicle_command_publisher.publish(msg)
	
	def PositionCallback(self, msg):
		self.position[0, 0] = msg.x
		self.position[0, 1] = msg.y
		self.position[0, 2] = msg.z
		
		self.velocity[0, 0] = msg.vx
		self.velocity[0, 1] = msg.vy
		self.velocity[0, 2] = msg.vz
		
		self.heading = msg.heading	
	
	def CommandAcknowledge(self, msg):
		#self.heading = msg.heading
		pass
	
	def ControlModeCallback(self, msg):
		self.is_armed = msg.flag_armed
		
	def AttitudeCallback(self, msg):
		orientation_q = msg.q

		self.quaternion = orientation_q
		
	def publish_thrust(self, thrust):
		msg = VehicleThrustSetpoint()
		
		msg.timestamp = int(Clock().now().nanoseconds / 1000)
		msg.xyz = np.array([0, 0, 1], dtype = np.float32)
	
		self.thrust_publisher.publish(msg)
		
	def publish_motor(self, thrusts):
		#msg = ActuatorMotors()
		
		#msg.timestamp = int(Clock().now().nanoseconds / 1000)
		#msg.control[0] = thrusts[0] * 0
		#msg.control[1] = thrusts[1] * 0
		#msg.control[2] = thrusts[2] * 0
		#msg.control[3] = thrusts[3] * 0
		
		#msg.control[4] = thrusts[0]
		#msg.control[5] = thrusts[0]
		#msg.control[6] = thrusts[0]
		#msg.control[7] = thrusts[0]
		#msg.control[8] = thrusts[0]
		#msg.control[9] = thrusts[0]
		#msg.control[10] = thrusts[0]
		#msg.control[11] = thrusts[0]
		
		#self.motor_publisher.publish(msg)
		
		msg = ActuatorServos()
		msg.timestamp = int(Clock().now().nanoseconds / 1000)
		#msg.control = np.zeros(8, dtype = np.float32) + thrusts[0]
		#msg.control[5] = thrusts[0]
		msg.control = msg.control + (thrusts[0] * 1)
		
		self.servo_publisher.publish(msg)
		