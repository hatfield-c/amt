import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import SensorGps
from px4_msgs.msg import VehicleCommand

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
		
		self.motor_publisher = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", qos_profile)

		self.quaternion = None
		self.heading = None

	def attitude_callback(self, msg):
		orientation_q = msg.q

		self.quaternion = orientation_q
		
	def gps_callback(self, msg):
		heading = msg.heading

		self.heading = heading

	def publish_motor(self, thrusts):
		msg = ActuatorMotors()
		
		msg.control = np.zeros(12, dtype = np.float32)
		msg.control[0] = thrusts[0]
		msg.control[1] = thrusts[1]
		msg.control[2] = thrusts[2]
		msg.control[3] = thrusts[3]
		
		self.motor_publisher.publish(msg)