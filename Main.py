
import time
import rclpy
import torch
import traceback

import RosController as RosController
import Quaternion
import Transform

try:

	rclpy.init()
	ros_controller = RosController.RosController()

	rclpy.spin(ros_controller)
	ros_controller.destroy_node()
	rclpy.shutdown()
except Exception as e:
	print(traceback.format_exc())