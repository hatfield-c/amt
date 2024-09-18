import sys
import time
import rclpy
import torch
import traceback

import RosController as RosController

is_log_stdout = False#True
is_write_video = True

def Main():
	rclpy.init()
	ros_controller = RosController.RosController(is_write_video = is_write_video)
	print("<uORB RosController Initialzed!>")

	rclpy.spin(ros_controller)
	ros_controller.destroy_node()
	rclpy.shutdown()
	
	ros_controller.depth_camera.data_pipe.stop()

	print("<uORB RosController Successfully closed.>")

if is_log_stdout:
	with open('/home/jetson/amt/data/log/tau_log.txt', "w") as sys.stdout:
		Main()
else:
	Main()
