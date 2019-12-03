#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from rosflight_msgs.msg import Command
from rosflight_msgs.msg import RCRaw
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

class hl_cmd_handler(object):
    def __init__(self):
        # Vars
        self.rc_msg = RCRaw()
        self.loop_rate = rospy.Rate(50)
        self.cmd_pub = rospy.Publisher('high_level_command', Command, queue_size=10)
        rospy.Subscriber('rc_raw', RCRaw, self.rc_cb)
        rospy.Subscriber('odom', Odometry, self.odom_cb)
        rospy.Subscriber('z_odom', Odometry, self.z_odom_cb)

        self.offb_switch = False
        self.rc_mode_chn = 5
        self.invert_yawrate_cmd = 1
        self.invert_yvel_cmd = 1
        self.invert_odom_yaw_angle = -1

        self.state = 'init'

        self.nearness_height_agl = -.5
        self.nearness_xvel = 1.0
        self.use_nearness_fwdspd = False
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
        self.hl_cmd.mode = 4
        self.hl_cmd.ignore = 15
        self.hl_cmd.x = 0.0
        self.hl_cmd.y = 0.0
        self.hl_cmd.z = 0.0
        self.hl_cmd.F = 0.0

        self.odom_msg = Odometry()
        self.initial_odom_state = self.odom_msg
        self.initial_yaw = 0.0
        self.odom_initialized = False
        self.yaw = 0.0

        self.pos_hold_cmd = Command()
        self.pos_hold_cmd.ignore = 0
        self.pos_hold_cmd.mode = 4
        self.pos_hold_initialized = False

        self.takeoff_initialized = False
        self.takeoff_ramp_time_s = 3
        self.takeoff_height_agl = self.nearness_height_agl
        self.takeoff_complete = False
        self.takeoff_thresh = .15

        self.landing_cmd = Command()
        self.landing_cmd.mode = 4
        self.landing_cmd.ignore = 0
        self.landing_cmd.x = 0.0
        self.landing_cmd.y = 0.0
        self.landing_cmd.z = 0.0
        self.landing_cmd.F = 0.1

        self.landing_F = .1
        self.landing_complete = False
        self.landing_initialized = False
        self.landing_ramp_time_s = 3
        self.landing_thresh = .15


    def rc_cb(self, data):
        self.rc_msg = data
        if(self.rc_msg.values[7] > 1500):
            rospy.loginfo_throttle(5,'Offboard control is enabled.')
            self.offb_switch = True
        else:
            self.offb_switch = False
            rospy.loginfo_throttle(5,'Offboard control is disabled.')

        if((self.rc_msg.values[self.rc_mode_chn] <= 1700) and (self.rc_msg.values[self.rc_mode_chn] > 1400)):
            if(not self.takeoff_complete):
                #rospy.loginfo_once("Control Mode: Takeoff")
                self.state = 'takeoff'
                if(not self.takeoff_initialized):
                    self.takeoff_start_time = rospy.Time.now()
                    self.takeoff_initialized = True
                    rospy.loginfo_once("Takeoff initialized.")
            elif(self.takeoff_complete and self.state == 'nearness_control'):
                self.state = 'pos_hold'

        if((self.rc_msg.values[self.rc_mode_chn] > 1700) and self.takeoff_complete):
            self.state = 'nearness_control'

        if(self.rc_msg.values[self.rc_mode_chn] <= 1400):
            rospy.loginfo_throttle(5, 'Control Mode: RC passthrough')
            if(self.state == 'pos_hold'):
                self.state = 'landing'


    def nearness_ctrl_cb(self, data):
        # Import nearness controller commands
        self.nearness_yvel = data.twist.linear.y
        if(self.use_nearness_fwdspd):
            self.nearness_xvel = data.twist.linear.x
        self.nearness_yawrate = data.twist.angular.z

    def z_odom_cb(self, data):
        rospy.loginfo_once("Z State received.")
        self.pos_z = -data.pose.pose.position.z

    def odom_cb(self, data):
        rospy.loginfo_once("Odometry received.")
        self.odom_msg = data
        self.pos_x = self.odom_msg.pose.pose.position.x
        self.pos_y = -self.odom_msg.pose.pose.position.y

        odom_msg_q = self.initial_odom_state.pose.pose.orientation
        odom_msg_q_list = [odom_msg_q.x, odom_msg_q.y, odom_msg_q.z, odom_msg_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(odom_msg_q_list)
        self.yaw = self.invert_odom_yaw_angle*self.yaw

        if(self.state == 'init' and not self.odom_initialized):
            self.initial_odom_state = self.odom_msg
            odom_q = self.initial_odom_state.pose.pose.orientation
            odom_q_list = [odom_q.x, odom_q.y, odom_q.z, odom_q.w]
            (roll, pitch, self.initial_yaw) = euler_from_quaternion(odom_q_list)
            self.initial_yaw = self.invert_odom_yaw_angle*self.initial_yaw
            self.odom_initialized = True
            rospy.loginfo_once("Initial Odometry state has been set.")

    def start(self):
        while not rospy.is_shutdown():
            if(self.offb_switch):
                if(self.state == 'init'):
                    rospy.loginfo_once("Control Mode: Init")
                    self.hl_cmd.ignore = 4
                    self.hl_cmd.mode = 4
                    self.hl_cmd.x = 0.0
                    self.hl_cmd.y = 0.0
                    self.hl_cmd.z = self.initial_yaw
                    self.hl_cmd.F = 1.0

                if(self.state == 'takeoff'):
                    rospy.loginfo_once("Control Mode: Takeoff")
                    self.hl_cmd.ignore = 0
                    self.hl_cmd.mode = 4
                    self.hl_cmd.x = self.initial_odom_state.pose.pose.position.x
                    self.hl_cmd.y = self.initial_odom_state.pose.pose.position.y
                    self.hl_cmd.z = self.initial_yaw
                    takeoff_time_s = (rospy.Time.now() - self.takeoff_start_time).to_sec()
                    #ramp = (takeoff_time_s/self.takeoff_ramp_time_s)*self.takeoff_height_agl
                    self.hl_cmd.F = np.clip((takeoff_time_s/self.takeoff_ramp_time_s)*self.takeoff_height_agl, self.takeoff_height_agl, 0.0)
                    #rospy.loginfo("Height error: %f", abs(self.pos_z - self.takeoff_height_agl))
                    if(abs(self.pos_z - self.takeoff_height_agl) < self.takeoff_thresh):
                        self.state = 'pos_hold'
                        self.takeoff_complete = True
                        rospy.loginfo_once("Takeoff completed.")

                if(self.state == 'pos_hold'):
                    rospy.loginfo_once("Control Mode: Pos. Hold")
                    if(not self.pos_hold_initialized):
                        self.pos_hold_cmd.x = self.pos_x
                        self.pos_hold_cmd.y = self.pos_y
                        self.pos_hold_cmd.z = self.yaw
                        self.pos_hold_cmd.F = self.nearness_height_agl
                        self.pos_hold_initialized = True
                    self.hl_cmd.ignore = self.pos_hold_cmd.ignore
                    self.hl_cmd.mode = self.pos_hold_cmd.mode
                    self.hl_cmd.x = self.pos_hold_cmd.x
                    self.hl_cmd.y = self.pos_hold_cmd.y
                    self.hl_cmd.z = self.pos_hold_cmd.z
                    self.hl_cmd.F = self.pos_hold_cmd.F
                else:
                    self.pos_hold_initialized = False

                if(self.state == 'nearness_control'):
                    rospy.loginfo_once("Control Mode: Nearness Control")
                    self.hl_cmd.ignore = 0
                    self.hl_cmd.mode = 5
                    self.hl_cmd.x = self.nearness_xvel
                    #self.hl_cmd.y = self.invert_xvel_cmd*self.nearness_yvel
                    self.hl_cmd.y = 0.0
                    self.hl_cmd.z = 0.0
                    #self.hl_cmd.z = self.nearness_yawrate
                    self.hl_cmd.F = self.nearness_height_agl

                if(self.state == 'landing'):
                    rospy.loginfo_once("Control Mode: Landing")
                    self.hl_cmd.ignore = 0
                    self.hl_cmd.mode = 4
                    if(not self.landing_initialized):
                        self.landing_cmd.x = self.pos_x
                        self.landing_cmd.y = self.pos_y
                        self.landing_cmd.z = self.yaw
                        self.landing_cmd.F = self.pos_z
                        self.landing_start_time = rospy.Time.now()
                        self.landing_initialized = True

                    self.hl_cmd.x = self.landing_cmd.x
                    self.hl_cmd.y = self.landing_cmd.y
                    self.hl_cmd.z = self.landing_cmd.z

                    landing_time_s = (rospy.Time.now() - self.landing_start_time).to_sec()
                    self.hl_cmd.F = np.clip((1-landing_time_s/self.landing_ramp_time_s), 0, 1)*self.landing_cmd.F

                    if(abs(self.pos_z) < self.landing_thresh):
                        self.state = 'landed'
                        self.landing_complete = True
                        self.takeoff_complete = False

                if(self.state == 'landed'):
                    self.hl_cmd.ignore = 4
                    self.hl_cmd.mode = 4
                    self.hl_cmd.x = 0.0
                    self.hl_cmd.y = 0.0
                    self.hl_cmd.z = 0.0
                    self.hl_cmd.F = .5

                self.cmd_pub.publish(self.hl_cmd);
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hl_cmd_handler', anonymous=True)
    hl_cmd_node = hl_cmd_handler()
    hl_cmd_node.start()
