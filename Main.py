import sys
import time
import rclpy
import torch
import traceback

import RosController as RosController

#with open('data/log/tau_log.txt', "w") as sys.stdout:
try:

	rclpy.init()
	ros_controller = RosController.RosController()
	print("<uORB RosController Initialzed!>")

	rclpy.spin(ros_controller)
	ros_controller.destroy_node()
	rclpy.shutdown()
except Exception as e:
	print("Exception occured!")
	print(traceback.format_exc())