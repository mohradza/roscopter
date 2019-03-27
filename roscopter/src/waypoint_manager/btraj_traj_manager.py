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
        self.pos_cmd = PositionCommand()
        self.loop_rate = rospy.Rate(20)
        self.cmd_pub = rospy.Publisher('high_level_command', Command, queue_size=10)
        self.btraj_cmd_pub = rospy.Publisher('btraj_command', BtrajCommand, queue_size=10)
        self.btraj_goalpt_pub = rospy.Publisher('waypoint_generator/waypoints', PoseStamped, queue_size=10)
        rospy.Subscriber('rc_raw', RCRaw, self.rc_cb)
        rospy.Subscriber('position_cmd', PositionCommand, self.pos_cmd_cb)

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


        # All outgoing commands should be 
        # published in the octomap / vicon frame
        # NED conversion handled in controller
        self.goal_point = PoseStamped()
        self.goal_point.pose.position.x = 7
        self.goal_point.pose.position.y = 0.0
        self.goal_point.pose.position.z = .75
        self.gp_switch = 0

        self.home_cmd = BtrajCommand()
        self.home_cmd.ignore = 0
        self.home_cmd.mode = 4
        self.home_cmd.controller_select = 1
        self.home_cmd.x = 1.2
        self.home_cmd.y = -1.6
        self.home_cmd.z = 0
        self.home_cmd.F = 0.75

        self.man_cmd = BtrajCommand()
        self.man_cmd.ignore = 7
        self.man_cmd.mode = 4
        self.man_cmd.controller_select = 1
        self.man_cmd.x = 0.0
        self.man_cmd.y = 0.0
        self.man_cmd.z = 0.0
        self.man_cmd.F = 0.75
        
        self.btraj_command_msg = BtrajCommand()
        self.btraj_command_msg.ignore = 0
        self.btraj_command_msg.mode = 4
        self.btraj_command_msg.controller_select = 2

    def rc_cb(self, data):
        self.rc_msg = data
        #rospy.loginfo("6: %d, 7: %d, 8: %d", self.rc_msg.values[5], self.rc_msg.values[6],self.rc_msg.values[7])

    def pos_cmd_cb(self, data):
        self.pos_cmd = data

    def start(self):
        command_msg1 = BtrajCommand()
        command_msg2 = BtrajCommand()
        while not rospy.is_shutdown():

            if(self.pos_cmd.trajectory_flag == 0 and self.rc_msg.values[6] < 1500):
                rospy.loginfo_throttle(1, 'Commanding home position')
                command_msg1 = self.home_cmd
                self.btraj_cmd_pub.publish(command_msg1);
            elif(self.rc_msg.values[6] > 1500):
                rospy.loginfo_throttle(1, "Manual roll, pitch, yaw override!")
                self.btraj_cmd_pub.publish(self.man_cmd)
            elif((self.pos_cmd.trajectory_flag == 1 or self.pos_cmd.trajectory_flag == 3) and self.rc_msg.values[6] < 1500):
                rospy.loginfo_throttle(1, 'Commanding trajectory')
                command_msg2.x = self.pos_cmd.position.x
                command_msg2.y = self.pos_cmd.position.y
                command_msg2.F = self.pos_cmd.position.z
                command_msg2.z = 0.0;
                command_msg2.x_vel = self.pos_cmd.velocity.x
                command_msg2.y_vel = self.pos_cmd.velocity.y
                command_msg2.z_vel = self.pos_cmd.velocity.z
                command_msg2.x_acc = self.pos_cmd.acceleration.x
                command_msg2.y_acc = self.pos_cmd.acceleration.y
                command_msg2.z_acc = self.pos_cmd.acceleration.z
                command_msg2.controller_select = 2
                self.btraj_cmd_pub.publish(command_msg2);

            if((self.rc_msg.values[7] > 1500)):
                #rospy.loginfo("GP Sent")
                if(self.gp_switch == 0):
                    rospy.loginfo("GP Sent")
                    self.btraj_goalpt_pub.publish(self.goal_point)
                    self.gp_switch = 1
                #rospy.loginfo("%d", self.gp_switch == False)
            else:
                self.gp_switch = 0

            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hl_cmd_handler', anonymous=True, disable_signals=True)
    hl_cmd_node = hl_cmd_handler()
    hl_cmd_node.start()
