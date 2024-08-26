import subprocess
import time
import rclpy

import RosController as RosController

rclpy.init()

ros_controller = RosController.RosController()

for i in range(100):
	rclpy.spin_once(ros_controller)
	print("main:", ros_controller.quaternion)
	
	time.sleep(0.7)

ros_controller.destroy_node()
rclpy.shutdown()

#from pymavlink import mavutil
#time_step = 1 / 20
#connection = mavutil.mavlink_connection("udpin:localhost:14569")
#connection.wait_heartbeat()
#print("Heartbeat for system (system %u component %u)" % (connection.target_system, connection.target_component))