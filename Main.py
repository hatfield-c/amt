import subprocess
import time
import rclpy
import torch

import RosController as RosController
import Quaternion
import Transform

from pymavlink import mavutil

connection = mavutil.mavlink_connection("udpin:localhost:14569")
connection.wait_heartbeat()
print("Heartbeat for system (system %u component %u)" % (connection.target_system, connection.target_component))

connection.mav.command_long_send(
	0, 
	0, 
	400, 
	0,
	1, 
	21196, 
	0, 
	0, 
	0, 
	0,
	0
)

connection.mav.command_long_send(
	0, 
	0, 
	187, 
	0,
	0.3, 
	0.3, 
	0.3, 
	0.3, 
	0.3, 
	0.3,
	0
)

#connection.set_actuator(0.3, 0.3, 0.3, 0.3, 0.3, 0.3)

exit()

rclpy.init()

ros_controller = RosController.RosController()

rclpy.spin(ros_controller)

ros_controller.SetArmed(0.0)
time.sleep(0.05)
ros_controller.destroy_node()
rclpy.shutdown()
