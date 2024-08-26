import subprocess
import time
import rclpy
import torch

import RosController as RosController
import Quaternion
import Transform

rclpy.init()

ros_controller = RosController.RosController()



for i in range(100):
	rclpy.spin_once(ros_controller)
	
	quaternion = torch.FloatTensor(ros_controller.quaternion).cuda()
	quaternion = quaternion.view(1, -1)
	
	local_up = Transform.GetUp(quaternion)
	local_forward = Transform.GetForward(quaternion)
	
	print("main:", local_up, local_forward, ros_controller.heading)
	input()
	
	time.sleep(0.7)

ros_controller.destroy_node()
rclpy.shutdown()
