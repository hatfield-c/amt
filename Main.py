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

exit()

rclpy.init()

ros_controller = RosController.RosController()

rclpy.spin(ros_controller)

ros_controller.SetArmed(0.0)
time.sleep(0.05)
ros_controller.destroy_node()
rclpy.shutdown()
