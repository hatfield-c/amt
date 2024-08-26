import subprocess
import time
import rclpy
import torch

import RosController as RosController
import Quaternion
import Transform

rclpy.init()

ros_controller = RosController.RosController()



for i in range(10):
	rclpy.spin_once(ros_controller)
	
	quaternion = torch.FloatTensor(ros_controller.quaternion).cuda()
	quaternion = quaternion.view(1, -1)
	
	local_up = Transform.GetUp(quaternion)
	local_forward = Transform.GetForward(quaternion)
	
	print("main:", local_up, local_forward, ros_controller.heading)
	#input()
	
	ros_controller.publish_motor([0, 0, 0, 0])
	
	time.sleep(1)

ros_controller.destroy_node()
rclpy.shutdown()
