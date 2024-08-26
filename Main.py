import subprocess
import time
import rclpy
import torch

import RosController as RosController
import Quaternion
import Transform

rclpy.init()

ros_controller = RosController.RosController()

ros_controller.SetArmed(1.0)
time.sleep(0.05)

for i in range(10):
	rclpy.spin_once(ros_controller)
	
	ros_controller.PrepareToCommand()
	
	#print("main:", local_up, local_forward, ros_controller.heading)
	#input()
	
	ros_controller.publish_thrust(1.0)
	ros_controller.publish_motor([0.1, 0.1, 0.1, 0.1])
	
	time.sleep(0.3)

ros_controller.SetArmed(0.0)
time.sleep(0.05)

ros_controller.destroy_node()
rclpy.shutdown()
