import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleAttitude
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
		
		#self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
		#self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

		print("ROS Init!")

	def attitude_callback(self, msg):
		print("attitude callback")
		orientation_q = msg.q

		#trueYaw is the drones current yaw value
		self.quaternion = orientation_q

	def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
		msg = VehicleCommand()
		msg.param1 = param1
		msg.param2 = param2
		msg.param7 = param7	# altitude value in takeoff command
		msg.command = command  # command ID
		msg.target_system = 1  # system which should execute the command
		msg.target_component = 1  # component which should execute the command, 0 for all components
		msg.source_system = 1  # system sending the command
		msg.source_component = 1  # component sending the command
		msg.from_external = True
		msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
		self.vehicle_command_publisher_.publish(msg)