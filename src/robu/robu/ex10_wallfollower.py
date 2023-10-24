import math
import rclpy

from std_msgs.msg import String
from geometry_msgs.msg import Twist 
from std_msgs.msg import LaserScan

from rclpy.qos import qos_profile_sensor_data

import numpy as np

from enum import IntEnum

class WallFollowerStates(IntEnum):
    WF_STATE_DETECTWALL = 0,
    WF_STATE_DRIVE2WALL = 1,
    WF_STATE_ROTATE2WALL = 2,
    WF_STATE_FOLLOWWALL = 3

class WallFollower(rclpy.Node):
    def __init__(self):
        super().__init__('WallFollower')
        self.scan_subscriber = self.create_subscription(LaserScan, "/scan",self.scan_callback,qos_profile_sensor_data)

    def scan_callback(self,msg):
        pass

def main(args=None):
    print('Hi from ehh.')



