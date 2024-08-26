import subprocess
import time
import rclpy

import RosController as RosController
import Quaternion
import Transform

rclpy.init()

ros_controller = RosController.RosController()

for i in range(100):
	rclpy.spin_once(ros_controller)
	
	quaternion = ros_controller.quaternion
	print(type(quaternion), quaternion.shape)
	local_up = Transform.GetUp(quaternion)
	local_forward = Transform.GetForward(quaternion)
	
	print("main:", local_up, local_forward)
	input()
	
	time.sleep(0.7)

ros_controller.destroy_node()
rclpy.shutdown()

#from pymavlink import mavutil
#time_step = 1 / 20
#connection = mavutil.mavlink_connection("udpin:localhost:14569")
#connection.wait_heartbeat()
#print("Heartbeat for system (system %u component %u)" % (connection.target_system, connection.target_component))