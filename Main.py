import sys
import time
import rclpy
import torch
import traceback

import RosController as RosController

with open('tau_log.txt') as sys.stdout:
	try:
	
		rclpy.init()
		ros_controller = RosController.RosController()
	
		rclpy.spin(ros_controller)
		ros_controller.destroy_node()
		rclpy.shutdown()
	except Exception as e:
		print("Exception occured!")
		print(traceback.format_exc())