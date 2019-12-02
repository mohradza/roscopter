#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from rosflight_msgs.msg import Command
from rosflight_msgs.msg import RCRaw
from nav_msgs.msg import Path

class hl_cmd_handler(object):
    def __init__(self):
        # Vars
        self.rc_msg = RCRaw()
        self.loop_rate = rospy.Rate(50)
        self.cmd_pub = rospy.Publisher('high_level_command', Command, queue_size=10)
        rospy.Subscriber('rc_raw', RCRaw, self.rc_cb)

        self.offb_switch = False
 

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

        self.hl_cmd = Command()
        self.hl_cmd.ignore = 7
        self.hl_cmd.mode = 4 
        self.hl_cmd.x = 0.0
        self.hl_cmd.y = 0.0
        self.hl_cmd.z = 0.0
        self.hl_cmd.F = -1.0

    def rc_cb(self, data):
        self.rc_msg = data
        if(self.rc_msg.values[5] > 1500):
            rospy.loginfo('switch')
            self.offb_switch = True
        else:
            self.offb_switch = False

    def start(self):
        while not rospy.is_shutdown():
            if(self.offb_switch):
                self.cmd_pub.publish(self.hl_cmd);
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hl_cmd_handler', anonymous=True)
    hl_cmd_node = hl_cmd_handler()
    hl_cmd_node.start()
