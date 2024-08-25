import subprocess
import time
import rclpy

import RosController as RosController

rclpy.init()

command = "micro-xrce-dds-agent udp4 -p 8888"
subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
time.sleep(1)

ros_controller = RosController.RosController()

for i in range(100):
	rclpy.spin_once(ros_controller)
	print(ros_controller.trueYaw)
	
	time.sleep(0.7)

ros_controller.destroy_node()
rclpy.shutdown()

#from pymavlink import mavutil
#time_step = 1 / 20
#connection = mavutil.mavlink_connection("udpin:localhost:14569")
#connection.wait_heartbeat()
#print("Heartbeat for system (system %u component %u)" % (connection.target_system, connection.target_component))