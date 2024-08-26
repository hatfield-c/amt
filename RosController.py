import time

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
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
			self.attitude_callback,
			qos_profile
		)
		
		self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
		self.motor_publisher = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", qos_profile)
		self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)
		
		#self.thrust_publisher = self.create_publisher(VehicleThrustSetpoint, "/fmu/in/vehicle_thrust_setpoint", qos_profile)

		self.quaternion = None
		self.heading = None
		

		timer_period = 0.02
		self.timer = self.create_timer(timer_period, self.Update)
		
		self.armed = False
		self.servo_delta = 0.05
		self.servo_val = 0
		
		self.cycles = 0
		self.max_cycles = 100
		
	def Update(self):
		self.SetArmed(1.0)
		self.PrepareToCommand()
		
		#if not self.armed:
		self.SetArmed(1.0)
			#self.armed = True
		
		#self.publish_thrust(1.0)
		#self.publish_motor([0.5, 0, 0, 0])
		
		#if self.cycles > self.max_cycles:
			#self.publish_motor([0.0, 0, 0, 0])
		#	self.SetArmed(0.0)
		#	exit()

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
		
	def attitude_callback(self, msg):
		orientation_q = msg.q

		self.quaternion = orientation_q
		
	def publish_thrust(self, thrust):
		msg = VehicleThrustSetpoint()
		
		msg.timestamp = int(Clock().now().nanoseconds / 1000)
		msg.xyz = np.array([0, 0, 1], dtype = np.float32)
	
		self.thrust_publisher.publish(msg)
		
	def publish_motor(self, thrusts):
		msg = ActuatorMotors()
		
		msg.timestamp = int(Clock().now().nanoseconds / 1000)
		#msg.reversible_flags = 0
		msg.control = np.zeros(12, dtype = np.float32)
		msg.control[0] = thrusts[0]
		msg.control[1] = thrusts[1]
		msg.control[2] = thrusts[2]
		msg.control[3] = thrusts[3]
		
		if self.servo_val > 1:
			self.servo_val = 1
			self.servo_delta = -self.servo_delta
			
		if self.servo_val < 0:
			self.servo_val = 0
			self.servo_delta = -self.servo_delta
		
		self.servo_val += self.servo_delta
		
		#msg.control[4] = self.servo_val
		#msg.control[5] = self.servo_val
		#msg.control[6] = self.servo_val
		#msg.control[7] = self.servo_val
		#msg.control[8] = self.servo_val
		
		self.motor_publisher.publish(msg)