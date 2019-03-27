#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from rosflight_msgs.msg import Command
from rosflight_msgs.msg import BtrajCommand
from rosflight_msgs.msg import RCRaw
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Path, Odometry

class hl_cmd_handler(object):
    def __init__(self):
        # Vars
        self.rc_msg = RCRaw()
        self.loop_rate = rospy.Rate(50)
        self.cmd_pub = rospy.Publisher('high_level_command', Command, queue_size=10)
        self.btraj_cmd_pub = rospy.Publisher('btraj_command', BtrajCommand, queue_size=10)
        rospy.Subscriber('rc_raw', RCRaw, self.rc_cb)
        rospy.Subscriber('/vins_estimator/odometry', Odometry, self.odom_cb)

        self.switch = False

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_z = 0.0

        # THESE NEED TO BE SENT IN NWU
        # TO BE CONSISTENT WITH BTRAJ
        self.home_cmd = BtrajCommand()
        self.home_cmd.ignore = 0
        self.home_cmd.mode = 4
        self.home_cmd.controller_select = 1
        self.home_cmd.x = 0.0
        self.home_cmd.y = 0.0
        self.home_cmd.z = 0.0
        self.home_cmd.F = 1.0

        self.wp_cmd = BtrajCommand()
        self.wp_cmd.ignore = 0
        self.wp_cmd.mode = 4
        self.wp_cmd.controller_select = 1
        self.wp_cmd.x = 0.0
        self.wp_cmd.y = 0.0
        self.wp_cmd.z = 0.0
        self.wp_cmd.F = 1.0

        self.wp_cmd2 = BtrajCommand()
        self.wp_cmd2.ignore = 0
        self.wp_cmd2.mode = 4
        self.wp_cmd2.controller_select = 1
        self.wp_cmd2.x = 0.0
        self.wp_cmd2.y = -1.0
        self.wp_cmd2.z = 0.0
        self.wp_cmd2.F = 1.0

    def rc_cb(self, data):
        self.rc_msg = data
    
    def odom_cb(self, data):
        self.odom_x = data.pose.pose.position.x
        self.odom_y = data.pose.pose.position.y
        self.odom_z = data.pose.pose.position.z

    def start(self):
        command_msg1 = BtrajCommand()
        command_msg2 = BtrajCommand()
        command_msg3 = BtrajCommand()
        while not rospy.is_shutdown():

            if(self.rc_msg.values[6] < 1250):
                rospy.loginfo_throttle(1, 'Commanding home position')
                command_msg1 = self.home_cmd
                command_msg1.header.stamp = rospy.Time.now()
                self.btraj_cmd_pub.publish(command_msg1)
            elif((self.rc_msg.values[6] > 1250) and (self.rc_msg.values[6] < 1750)):
                rospy.loginfo_throttle(1, 'Commanding waypoints')
                command_msg2 = self.wp_cmd2
                command_msg2.header.stamp = rospy.Time.now()
                self.btraj_cmd_pub.publish(command_msg2)
            elif((self.rc_msg.values[6] < 2000) and (self.rc_msg.values[6] > 1750)):
                rospy.loginfo_throttle(1, 'Commanding waypoints2')
                command_msg3 = self.wp_cmd2
                command_msg3.header.stamp = rospy.Time.now()
                self.btraj_cmd_pub.publish(command_msg3)
                
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hl_cmd_handler', anonymous=True, disable_signals=True)
    hl_cmd_node = hl_cmd_handler()
    hl_cmd_node.start()
