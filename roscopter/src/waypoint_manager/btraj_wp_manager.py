#!/usr/bin/env python

import numpy as np
import rospy, tf
from geometry_msgs.msg import PoseStamped
from rosflight_msgs.msg import Command
from rosflight_msgs.msg import BtrajCommand
from rosflight_msgs.msg import RCRaw
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

class hl_cmd_handler(object):
    def __init__(self):


        # get Waypoints for Automatic Take Off
        try:
            self.waypoint_list = rospy.get_param('/waypoint_manager/waypoints')
        except KeyError:
            rospy.logfatal('waypoints not set')

        self.threshold = rospy.get_param('~threshold', 0.1)

        # Vars
        self.rc_msg = RCRaw()
        self.odom = Odometry()
        self.loop_rate = rospy.Rate(50)
        self.cmd_pub = rospy.Publisher('high_level_command', Command, queue_size=10)
        self.btraj_cmd_pub = rospy.Publisher('btraj_command', BtrajCommand, queue_size=10)
        rospy.Subscriber('rc_raw', RCRaw, self.rc_cb)
        self.odom_sub = rospy.Subscriber('vins_odom', Odometry, self.odometryCallback, queue_size=5)


        self.switch = False

#        try:
#            self.waypoint_list = rospy.get_param('/waypoint_manager/waypoints')
#        except KeyError:
#            rospy.logfatal('waypoints not set')
#            rospy.signal_shutdown('Parameters not set')
#        self.current_waypoint_index = 0
#        self.end_index = len(self.waypoint_list)-1

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
        self.wp_cmd.z = -0.5
        self.wp_cmd.F = 1.0

        self.wp_cmd2 = BtrajCommand()
        self.wp_cmd2.ignore = 0
        self.wp_cmd2.mode = 4
        self.wp_cmd2.controller_select = 1
        self.wp_cmd2.x = 0.0
        self.wp_cmd2.y = -1.0
        self.wp_cmd2.z = -0.5
        self.wp_cmd2.F = 1.0

    def rc_cb(self, data):
        self.rc_msg = data

    def start(self):
        command_msg1 = BtrajCommand()
        command_msg2 = BtrajCommand()
        command_msg3 = BtrajCommand()

        self.take_off = 1
        self.current_waypoint_index = 0

        takeoff_msg = BtrajCommand()
        current_waypoint = np.array(self.waypoint_list[0])
        takeoff_msg.x = current_waypoint[0]
        takeoff_msg.y = current_waypoint[1]
        takeoff_msg.z = current_waypoint[2]
        takeoff_msg.x_vel = current_waypoint[3]
        takeoff_msg.y_vel = current_waypoint[4]
        takeoff_msg.z_vel = current_waypoint[5]
        current_position = np.zeros(3)
        error = 100000000000000000

        while not rospy.is_shutdown():
            if(self.take_off == 1):
                current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
                (r, p, y) = tf.transformations.euler_from_quaternion([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])
                current_position = np.array([self.odom.pose.pose.position.x,
                                             self.odom.pose.pose.position.y,
                                             self.odom.pose.pose.position.z])
                error = np.linalg.norm(-current_position[2] - current_waypoint[3])

                if error < self.threshold:

                    # Get new waypoint index
                    self.current_waypoint_index += 1
                    next_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
                    takeoff_msg.x = next_waypoint[0]
                    takeoff_msg.y = next_waypoint[1]
                    takeoff_msg.z = next_waypoint[2]
                    takeoff_msg.x_vel = next_waypoint[3]
                    takeoff_msg.y_vel = next_waypoint[4]
                    takeoff_msg.z_vel = next_waypoint[5]

                    if(self.current_waypoint_index == np.shape(self.waypoint_list)[0] - 1):
                        self.take_off = 0

                rospy.loginfo_throttle(1, current_waypoint[3])
                rospy.loginfo_throttle(1, self.odom.pose.pose.position.z)
                rospy.loginfo_throttle(1, error)
                rospy.loginfo_throttle(1, self.current_waypoint_index)
                rospy.loginfo_throttle(1, self.take_off)


            else:
                rospy.loginfo("ahh")
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

    def odometryCallback(self, msg):
        # Get error between waypoint and current state
        self.odom = msg
    
if __name__ == '__main__':
    rospy.init_node('hl_cmd_handler', anonymous=True, disable_signals=True)
    hl_cmd_node = hl_cmd_handler()
    hl_cmd_node.start()
