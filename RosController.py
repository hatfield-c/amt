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
		
		self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
		self.motor_publisher = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", 10)
		self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
		
		self.thrust_publisher = self.create_publisher(VehicleThrustSetpoint, "/fmu/in/vehicle_thrust_setpoint", 10)

		self.quaternion = None
		self.heading = None

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
			time.sleep(0.05)
			
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
		
		#msg.timestamp_sample = int(Clock().now().nanoseconds / 1000)
		msg.timestamp = int(Clock().now().nanoseconds / 1000)
		msg.reversible_flags = 0
		msg.control = np.zeros(12, dtype = np.float32) + 1.0
		#msg.control[0] = thrusts[0]
		#msg.control[1] = thrusts[1]
		#msg.control[2] = thrusts[2]
		#msg.control[3] = thrusts[3]
		
		self.motor_publisher.publish(msg)