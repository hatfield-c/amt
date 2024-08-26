import subprocess
import time
import rclpy
import torch

import RosController as RosController
import Quaternion
import Transform

rclpy.init()

ros_controller = RosController.RosController()

rclpy.spin(ros_controller)

ros_controller.SetArmed(0.0)
time.sleep(0.05)
ros_controller.destroy_node()
rclpy.shutdown()
