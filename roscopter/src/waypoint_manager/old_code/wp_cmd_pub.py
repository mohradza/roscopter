#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from rosflight_msgs.msg import Command
from rosflight_msgs.msg import BtrajCommand
from rosflight_msgs.msg import RCRaw
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Path

class hl_cmd_handler(object):
    def __init__(self):
        # Vars
        self.rc_msg = RCRaw()
        self.loop_rate = rospy.Rate(20)
        self.cmd_pub = rospy.Publisher('high_level_command', Command, queue_size=10)
        self.btraj_cmd_pub = rospy.Publisher('btraj_command', BtrajCommand, queue_size=10)
        rospy.Subscriber('rc_raw', RCRaw, self.rc_cb)

        self.switch = False

        try:
            self.waypoint_list = rospy.get_param('/waypoint_manager/waypoints')
        except KeyError:
            rospy.logfatal('waypoints not set')
            rospy.signal_shutdown('Parameters not set')
        self.current_waypoint_index = 0
        self.end_index = len(self.waypoint_list)-1

        # This should be in vehicle NED frame
        #uint8 MODE_PASS_THROUGH = 0
        #uint8 MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE = 1
        #uint8 MODE_ROLL_PITCH_YAWRATE_THROTTLE = 2
        #uint8 MODE_ROLL_PITCH_YAWRATE_ALTITUDE = 3
        #uint8 MODE_XPOS_YPOS_YAW_ALTITUDE = 4
        #uint8 MODE_XVEL_YVEL_YAWRATE_ALTITUDE = 5
        #uint8 MODE_XACC_YACC_YAWRATE_AZ = 6

        #uint8 IGNORE_NONE = 0
        #uint8 IGNORE_X = 1
        #uint8 IGNORE_Y = 2
        #uint8 IGNORE_Z = 4
        #uint8 IGNORE_F = 8

        self.home_cmd = BtrajCommand()
        self.home_cmd.ignore = 0
        self.home_cmd.mode = 4
        self.home_cmd.controller_select = 1
        self.home_cmd.x = 0.0
        self.home_cmd.y = 0.0
        self.home_cmd.z = 0.0
        self.home_cmd.F = -0.75

        self.home_cmd1 = BtrajCommand()
        self.home_cmd1.ignore = 0
        self.home_cmd1.mode = 4
        self.home_cmd1.controller_select = 1
        self.home_cmd1.x = 1.0
        self.home_cmd1.y = 0.0
        self.home_cmd1.z = 0.0
        self.home_cmd1.F = -.75

    def rc_cb(self, data):
        self.rc_msg = data

    def start(self):
        while not rospy.is_shutdown():

            if(self.rc_msg.values[6] < 1500):
                rospy.loginfo_throttle(1, 'Commanding home position')
                self.btraj_cmd_pub.publish(self.home_cmd)
            else:
                rospy.loginfo_throttle(1, 'Commanding waypoint 1')
                self.btraj_cmd_pub.publish(self.home_cmd1)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hl_cmd_handler', anonymous=True, disable_signals=True)
    hl_cmd_node = hl_cmd_handler()
    hl_cmd_node.start()
