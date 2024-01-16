import math
import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data #um

import numpy as np

from enum import IntEnum

ROBOT_DIRECTION_FRONT_INDEX = 0
ROBOT_DIRECTION_RIGHT_FRONT_INDEX = 300
ROBOT_DIRECTION_RIGHT_INDEX = 270
ROBOT_DIRECTION_RIGHT_REAR_INDEX = 240
ROBOT_DIRECTION_REAR_INDEX = 180
ROBOT_DIRECTION_LEFT_REAR_INDEX = 120
ROBOT_DIRECTION_LEFT_INDEX = 90
ROBOT_DIRECTION_LEFT_FRONT_INDEX = 60


class WallFollowerStates(IntEnum):
    WF_STATE_DETECTWALL = 0,
    WF_STATE_DRIVE2WALL = 1,
    WF_STATE_ROTATE2WALL = 2,
    WF_STATE_FOLLOWWALL = 3

class WallFollower(Node):
    def __init__(self):
        super().__init__('WallFollower')
        self.scan_subscriber = self.create_subscription(LaserScan, "/scan",self.scan_callback,qos_profile_sensor_data)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.left_dist = 99999999.9
        self.front_dist = 99999999.9
        self.leftfront_dist = 99999999.9
        self.right_dist = 99999999.9
        self.rightfront_dist = 99999999.9
        self.rear_dist = 99999999.9
        self.distances = []

        self.wallfollower_state = WallFollowerStates.WF_STATE_DETECTWALL
        
        self.forward_speed_wf_slow = 0.05
        self.forward_speed_wf_fast = 0.1

        self.turning_speed_wf_slow = 0.1
        self.turning_speed_wf_fast = 1.0

        self.dist_laser_offset = 0.03

        self.dist_treshhold_wf = 0.3
        self.dist_hysteresis_wf = 0.02
        self.valid_lidar_data = False

        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        if self.valid_lidar_data:
            self.follow_wall()

    def follow_wall(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        if self.wallfollower_state == WallFollowerStates.WF_STATE_DETECTWALL:
            print("Hallo 1")
            dist_min = min(self.distances)
            if self.front_dist > (dist_min + self.dist_laser_offset):
                print("Hallo 2 - Drehe mich")
                if abs(self.front_dist - dist_min) <0.2:
                    msg.angular.z = self.turning_speed_wf_slow
                else: 
                    msg.angular.z = self.turning_speed_wf_fast
            else:
                print("WF_STATE_DRIVE2WALL")
                self.wallfollower_state = WallFollowerStates.WF_STATE_DRIVE2WALL
        elif self.wallfollower_state == WallFollowerStates.WF_STATE_DRIVE2WALL:
            print("Kommt als nächstes")

        print("Hallo 3: Nachricht an Roboter senden")
        self.cmd_vel_publisher.publish(msg)



    def scan_callback(self,msg):
        self.left_dist = msg.ranges[ROBOT_DIRECTION_LEFT_INDEX]
        self.front_dist = msg.ranges[ROBOT_DIRECTION_FRONT_INDEX]
        self.leftfront_dist = msg.ranges[ROBOT_DIRECTION_LEFT_FRONT_INDEX]
        self.right_dist = msg.ranges[ROBOT_DIRECTION_RIGHT_INDEX]
        self.rightfront_dist = msg.ranges[ROBOT_DIRECTION_RIGHT_FRONT_INDEX]
        self.rear_dist = msg.ranges[ROBOT_DIRECTION_REAR_INDEX]

        self.distances = msg.ranges

        print("ld: %.2f m\n"% self.left_dist,
              "lfd: %.2f m\n"%self.leftfront_dist,
              "fd: %.2f m\n"%self.front_dist,
              "rfd: %.2f m\n"%self.rightfront_dist,
              "rd: %.2f m\n"% self.right_dist,
              "rrd: %.2f m\n"% self.rear_dist)
        
        self.valid_lidar_data=True

    def calc_linear_speed(self):
        fd_thresh = self.dist_treshhold_wf + self.dist_laser_offset
        if self.front_dist> (1.2 * fd_thresh):
            forward_speed_wf = self.forward_speed_wf_fast
        else:
            forward_speed_wf = self.forward_speed_wf_slow

        return forward_speed_wf
    
    def align_front(self):
        fl = self.distances[ROBOT_DIRECTION_LEFT_FRONT_INDEX]
        fr = self.distances[ROBOT_DIRECTION_RIGHT_FRONT_INDEX]

        if (fl-fr)> self.dist_hysteresis_wf:
            return 1
        elif (fr-fl) > self.dist_hysteresis_wf:
            return -1
        else:
            return 0

     


def main(args=None):
    rclpy.init(args = args)
    wallfollower= WallFollower()

    rclpy.spin(wallfollower)

    wallfollower.destroy_node()
    rclpy.shutdown()
    



