#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from rosflight_msgs.msg import Command, BtrajCommand, RCRaw, ControlStatus
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Path, Odometry
from z_state_estimator.msg import ZStateEst
from marble_common.msg import Estop

import tf
import math

class hl_cmd_handler(object):
    def __init__(self):

        # Define Variables
        self.rc_msg = RCRaw()
        self.pos_cmd = PositionCommand()
        self.control_status = ControlStatus()

        self.loop_rate = rospy.Rate(20)
        self.height_des = .75

        # Define Publishers
        self.cmd_pub = rospy.Publisher('high_level_command', Command, queue_size=10)
        self.btraj_cmd_pub = rospy.Publisher('btraj_command', BtrajCommand, queue_size=10)
        self.btraj_goalpt_pub = rospy.Publisher('goal', PoseStamped, queue_size=10)
        self.control_status_pub = rospy.Publisher('control_status', ControlStatus, queue_size=10)

        # Define Subscribers
        rospy.Subscriber('goal', PoseStamped, self.goalpt_cb)
        rospy.Subscriber('rc_raw', RCRaw, self.rc_cb)
        rospy.Subscriber('position_cmd', PositionCommand, self.pos_cmd_cb)
        rospy.Subscriber('vins_estimator/odometry', Odometry, self.vins_odom_cb)
        rospy.Subscriber('z_state_estimator/z_state_estimate', ZStateEst, self.z_state_cb)
        rospy.Subscriber('estop', Estop, self.estop_cb)
        
        # Initialize Control Status
        self.control_status.control_status = 0
        
        # Initialize Control Status
        self.STATE_LANDED_ = True
        self.STATE_TAKEOFF_ = False
        self.STATE_HOVER_ = False
        self.STATE_TRAJ_ = False
        self.STATE_TRAJ_TO_HOVER_ = False
        self.STATE_LANDING_ = False

        self.CMD_AUTO_ = False
        self.CMD_MANUAL_ = False
        self.CMD_ESTOP_ = False
        self.CMD_TRAJ_ = False
        self.CMD_HOVER_ = False
        self.MEAS_HOVER_ = False
        self.MEAS_LANDED_ = False

        # Initialize Take Off Switch
        self.RC_SWITCH_AUTONOMOUS_ = 0
        self.RC_SWITCH_TEST_ = 0
        self.RC_SWITCH_TEST2_ = 0
        
        # Initialize Replan
        self.control_status.replan = 0

        # More Initialization
        self.initial_turn = True
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.end_yaw = 0.0
        self.end_psi_des = 0.0

        self.vins_odom = Odometry()

        # All outgoing commands should be 
        # published in the octomap / vicon frame
        # which is NWU.
        # NED conversion handled in controller
        self.goal_point = PoseStamped()
        #self.goal_point.pose.position.x = 2.0
        #self.goal_point.pose.position.y = 2.0
        #self.goal_point.pose.position.z = .75
        self.gp_switch = True
        self.gp_reached = False
        self.gp_thresh = 1.5       # Euclidean distance to goalpointi (m)
        self.end_traj_switch = True

        self.home_cmd = BtrajCommand()
        self.home_cmd.ignore = 0
        self.home_cmd.mode = 4
        self.home_cmd.controller_select = 2
        self.home_cmd.x = 0.0
        self.home_cmd.y = 0.0
        self.home_cmd.z = 0.0
        self.home_cmd.F = self.height_des

        self.man_cmd = BtrajCommand()
        self.man_cmd.ignore = 7
        self.man_cmd.mode = 4
        self.man_cmd.controller_select = 1
        self.man_cmd.x = 0.0
        self.man_cmd.y = 0.0
        self.man_cmd.z = 0.0
        self.man_cmd.F = self.height_des
        
        self.turn_mnvr = BtrajCommand()
        self.turn_mnvr.ignore = 0
        self.turn_mnvr.mode = 4
        self.turn_mnvr.controller_select = 2
        self.turn_mnvr.x = 0.0
        self.turn_mnvr.y = 0.0
        self.turn_mnvr.z = 0.0
        self.turn_mnvr.F = self.height_des
        self.turn_switch = True
        self.turn_maneuver = False

        self.log_throttle_rate = 1 # seconds

    def estop_cb(self, msg):

        if msg.cmd == 1:
            self.CMD_ESTOP_ = True
        else: 
            self.CMD_ESTOP_ = False
        
        rospy.loginfo_throttle(self.log_throttle_rate, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
        rospy.loginfo_throttle(self.log_throttle_rate, "CMD_ESTOP_:")
        rospy.loginfo_throttle(self.log_throttle_rate, self.CMD_ESTOP_)

    def rc_cb(self, msg):
        self.rc_msg = msg

        self.RC_SWITCH_AUTONOMOUS_ = msg.values[7]

        if(self.RC_SWITCH_AUTONOMOUS_ > 1750):
            self.CMD_AUTO_ = True
            self.CMD_MANUAL_ = False
        else:
            self.CMD_AUTO_ = False
            self.CMD_MANUAL_ = True

        # rospy.loginfo_throttle(self.log_throttle_rate, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
        # rospy.loginfo_throttle(self.log_throttle_rate, "RC_SWITCH_AUTONOMOUS_:")
        # rospy.loginfo_throttle(self.log_throttle_rate, self.RC_SWITCH_AUTONOMOUS_)
        # rospy.loginfo_throttle(self.log_throttle_rate, "CMD_AUTO_:")
        # rospy.loginfo_throttle(self.log_throttle_rate, self.CMD_AUTO_)
        # rospy.loginfo_throttle(self.log_throttle_rate, "CMD_MANUAL_:")
        # rospy.loginfo_throttle(self.log_throttle_rate, self.CMD_MANUAL_)

        # self.RC_SWITCH_TEST_ = msg.values[6]

        # if(self.RC_SWITCH_TEST_ > 1500):
        #     self.MEAS_HOVER_ = True
        # else:
        #     self.MEAS_HOVER_ = False

        # rospy.loginfo_throttle(self.log_throttle_rate, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
        # rospy.loginfo_throttle(self.log_throttle_rate, "RC_SWITCH_TEST_:")
        # rospy.loginfo_throttle(self.log_throttle_rate, self.RC_SWITCH_TEST_)
        # rospy.loginfo_throttle(self.log_throttle_rate, "MEAS_HOVER_:")
        # rospy.loginfo_throttle(self.log_throttle_rate, self.MEAS_HOVER_)

        # self.RC_SWITCH_TEST2_ = msg.values[4]

        # if(self.RC_SWITCH_TEST2_ > 1500):
        #     self.CMD_TRAJ_ = True
        # else:
        #     self.CMD_TRAJ_ = False

        # rospy.loginfo_throttle(self.log_throttle_rate, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
        # rospy.loginfo_throttle(self.log_throttle_rate, "RC_SWITCH_TEST2_:")
        # rospy.loginfo_throttle(self.log_throttle_rate, self.RC_SWITCH_TEST2_)
        # rospy.loginfo_throttle(self.log_throttle_rate, "CMD_TRAJ_:")
        # rospy.loginfo_throttle(self.log_throttle_rate, self.CMD_TRAJ_)

        # rospy.loginfo_throttle(self.log_throttle_rate, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
        # rospy.loginfo_throttle(self.log_throttle_rate, "RC_SWITCH_TEST_:")
        # rospy.loginfo_throttle(self.log_throttle_rate, self.RC_SWITCH_TEST_)
        # rospy.loginfo_throttle(self.log_throttle_rate, "MEAS_HOVER_:")
        # rospy.loginfo_throttle(self.log_throttle_rate, self.MEAS_HOVER_)

        # self.RC_SWITCH_TEST3_ = msg.values[2]

        # if(self.RC_SWITCH_TEST3_ > 1700):
        #     self.CMD_HOVER_ = True
        # else:
        #     self.CMD_HOVER_ = False

        # rospy.loginfo_throttle(self.log_throttle_rate, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
        # rospy.loginfo_throttle(self.log_throttle_rate, "RC_SWITCH_TEST3_:")
        # rospy.loginfo_throttle(self.log_throttle_rate, self.RC_SWITCH_TEST3_)
        # rospy.loginfo_throttle(self.log_throttle_rate, "CMD_HOVER_:")
        # rospy.loginfo_throttle(self.log_throttle_rate, self.CMD_HOVER_)

    def z_state_cb(self, msg):
        self.z_state_msg = msg

        # Handle switching to landed state
        if (self.z_state_msg.height_agl.data < 0.10 and self.z_state_msg.z_velocity.data < 1e-4):
            self.MEAS_LANDED_ = True
        else:
            self.MEAS_LANDED_ = False
        # Handle switching to hover from takeoff
        if (self.z_state_msg.height_agl.data > 0.5 and self.STATE_TAKEOFF_):
            self.MEAS_HOVER_ = True

        # rospy.loginfo_throttle(self.log_throttle_rate, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
        # rospy.loginfo_throttle(self.log_throttle_rate, "z_state_msg:")
        # rospy.loginfo_throttle(self.log_throttle_rate, self.z_state_msg.height_agl.data)
        # rospy.loginfo_throttle(self.log_throttle_rate, "z_velocity:")
        # rospy.loginfo_throttle(self.log_throttle_rate, self.z_state_msg.z_velocity.data)
        # rospy.loginfo_throttle(self.log_throttle_rate, "MEAS_LANDED_:")
        # rospy.loginfo_throttle(self.log_throttle_rate, self.MEAS_LANDED_)

    def pos_cmd_cb(self, msg):
        self.pos_cmd = msg

    def vins_odom_cb(self, msg):
        self.vins_odom = msg
        vins_quat = (msg.pose.pose.orientation.x, 
                     msg.pose.pose.orientation.y, 
                     msg.pose.pose.orientation.z, 
                     msg.pose.pose.orientation.w)
        vins_euler = tf.transformations.euler_from_quaternion(vins_quat)
        self.roll = vins_euler[0]
        self.pitch = vins_euler[1]
        self.yaw = vins_euler[2]

    def goalpt_cb(self, msg):
        self.goal_point = msg
        self.end_traj_switch = True
        self.new_goal = True
        self.gp_reached = False
        rospy.loginfo('Goal point received')
        x_pos_diff = self.goal_point.pose.position.x - self.vins_odom.pose.pose.position.x
        y_pos_diff = self.goal_point.pose.position.y - self.vins_odom.pose.pose.position.y
        self.end_yaw = math.atan2(y_pos_diff, x_pos_diff)


    def start(self):
        command_out  = BtrajCommand()
        command_out.ignore = 0
        command_out.mode = 4
        command_out.controller_select = 2

        while not rospy.is_shutdown():

            # STATE MACHINE

            self.STATE_LANDED_ = (self.STATE_LANDED_ or (self.STATE_LANDING_ and (self.CMD_MANUAL_ or self.CMD_ESTOP_) and self.MEAS_LANDED_)) and not self.STATE_TAKEOFF_
            self.STATE_TAKEOFF_ = (self.STATE_TAKEOFF_ or (self.STATE_LANDED_ and self.CMD_AUTO_)) and not self.STATE_LANDING_ and not self.STATE_HOVER_
            self.STATE_HOVER_ = (self.STATE_HOVER_ or (self.STATE_TAKEOFF_ and self.MEAS_HOVER_) or (self.STATE_TRAJ_TO_HOVER_ and self.MEAS_HOVER_)) and not self.STATE_TRAJ_ and not self.STATE_LANDING_
            self.STATE_TRAJ_ = (self.STATE_TRAJ_ or (self.STATE_HOVER_ and self.CMD_AUTO_ and self.CMD_TRAJ_)) and not self.STATE_LANDING_ and not self.STATE_TRAJ_TO_HOVER_ 
            self.STATE_TRAJ_TO_HOVER_ = (self.STATE_TRAJ_TO_HOVER_ or (self.STATE_TRAJ_ and self.CMD_AUTO_ and self.CMD_HOVER_)) and not self.STATE_HOVER_
            self.STATE_LANDING_ = (self.STATE_LANDING_ or (self.STATE_TAKEOFF_ and self.CMD_ESTOP_) or (self.STATE_HOVER_ and self.CMD_ESTOP_) or (self.STATE_TRAJ_ and self.CMD_ESTOP_) or (self.STATE_TAKEOFF_ and self.CMD_MANUAL_) or (self.STATE_HOVER_ and self.CMD_MANUAL_) or (self.STATE_TRAJ_ and self.CMD_MANUAL_)) and not self.STATE_LANDED_

            rospy.loginfo_throttle(self.log_throttle_rate, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
            rospy.loginfo_throttle(self.log_throttle_rate, "STATE_LANDED_:")
            rospy.loginfo_throttle(self.log_throttle_rate, self.STATE_LANDED_)
            rospy.loginfo_throttle(self.log_throttle_rate, "STATE_TAKEOFF_:")
            rospy.loginfo_throttle(self.log_throttle_rate, self.STATE_TAKEOFF_)
            rospy.loginfo_throttle(self.log_throttle_rate, "STATE_HOVER_:")
            rospy.loginfo_throttle(self.log_throttle_rate, self.STATE_HOVER_)
            rospy.loginfo_throttle(self.log_throttle_rate, "STATE_TRAJ_:")
            rospy.loginfo_throttle(self.log_throttle_rate, self.STATE_TRAJ_)
            rospy.loginfo_throttle(self.log_throttle_rate, "STATE_TRAJ_TO_HOVER_:")
            rospy.loginfo_throttle(self.log_throttle_rate, self.STATE_TRAJ_TO_HOVER_)
            rospy.loginfo_throttle(self.log_throttle_rate, "STATE_LANDING_:")
            rospy.loginfo_throttle(self.log_throttle_rate, self.STATE_LANDING_)

            # 1: LANDED
            if(self.STATE_LANDED_ and not self.STATE_TAKEOFF_ and not self.STATE_HOVER_ and not self.STATE_TRAJ_ and not self.STATE_TRAJ_TO_HOVER_ and not self.STATE_LANDING_):
                self.control_status.control_status = 0
            # 1: TAKEOFF
            elif(self.STATE_TAKEOFF_ and not self.STATE_HOVER_ and not self.STATE_TRAJ_ and not self.STATE_TRAJ_TO_HOVER_ and not self.STATE_LANDING_ and not self.STATE_LANDED_):
                self.control_status.control_status = 1
            # 2: HOVER
            elif(self.STATE_HOVER_ and not self.STATE_TRAJ_ and not self.STATE_TRAJ_TO_HOVER_ and not self.STATE_LANDING_ and not self.STATE_LANDED_ and not self.STATE_TAKEOFF_):
                self.control_status.control_status = 2
            # 3: TRAJECTORY
            elif(self.STATE_TRAJ_ and not self.STATE_TRAJ_TO_HOVER_ and not self.STATE_LANDING_ and not self.STATE_LANDED_ and not self.STATE_TAKEOFF_ and not self.STATE_HOVER_):
                self.control_status.control_status = 3
            # 4: TRAJECTORY TO HOVER
            elif(self.STATE_TRAJ_TO_HOVER_ and not self.STATE_LANDING_ and not self.STATE_LANDED_ and not self.STATE_TAKEOFF_ and not self.STATE_TRAJ_ and not self.STATE_HOVER_):
                self.control_status.control_status = 4
            # 5: LANDING
            elif(self.STATE_LANDING_ and not self.STATE_LANDED_ and not self.STATE_TAKEOFF_ and not self.STATE_HOVER_ and not self.STATE_TRAJ_ and not self.STATE_TRAJ_TO_HOVER_):
                self.control_status.control_status = 5
            # 6 : ERROR
            else:
                self.control_status.control_status = 6

            rospy.loginfo_throttle(self.log_throttle_rate, 'self.control_status.control_status')
            rospy.loginfo_throttle(self.log_throttle_rate, self.control_status.control_status)
            
            if (self.STATE_LANDED_):
                rospy.loginfo_throttle(self.log_throttle_rate, 'SHOULD PRINT 0')
            elif(self.STATE_TAKEOFF_):
                rospy.loginfo_throttle(self.log_throttle_rate, 'SHOULD PRINT 1')
            elif(self.STATE_HOVER_):
                rospy.loginfo_throttle(self.log_throttle_rate, 'SHOULD PRINT 2')
            elif(self.STATE_TRAJ_):
                rospy.loginfo_throttle(self.log_throttle_rate, 'SHOULD PRINT 3')
            elif(self.STATE_TRAJ_TO_HOVER_):
                rospy.loginfo_throttle(self.log_throttle_rate, 'SHOULD PRINT 4')
            elif(self.STATE_LANDING_):
                rospy.loginfo_throttle(self.log_throttle_rate, 'SHOULD PRINT 5')
            else:
                rospy.loginfo_throttle(self.log_throttle_rate, 'SHOULD PRINT 6')

            if (self.STATE_TAKEOFF_):# and not self.STATE_TAKEOFF_finished):
                pass
            elif(self.STATE_LANDED_):# and not self.landing_finished):
                pass
            elif(self.STATE_HOVER_ or self.STATE_TRAJ_):
                # Trajectory Flag = 0 ==> no trajectory available
                if(self.pos_cmd.trajectory_flag == 0 and self.rc_msg.values[6] < 1500):
                    rospy.loginfo_throttle(2, 'Commanding home position')
                    command_out = self.home_cmd
                    command_out.header.stamp = rospy.Time.now()
                    self.btraj_cmd_pub.publish(command_out)
                # Manual Control override from TX
                elif(self.rc_msg.values[6] > 1500):
                    rospy.loginfo_throttle(2, "Manual roll, pitch, yaw override!")
                    command_out = self.man_cmd
                    command_out.header.stamp = rospy.Time.now()
                    self.btraj_cmd_pub.publish(command_out)
                # If we have a trajectory from Btraj, follow it    
                elif((self.pos_cmd.trajectory_flag == 1 or self.pos_cmd.trajectory_flag == 3) and self.rc_msg.values[6] < 1500):
                    if(self.STATE_HOVER_ and self.pos_cmd.trajector_flag == 1):
                        self.CMD_TRAJ_ = True
                    # Use velocity vector instead
                    psi_des = math.atan2(self.pos_cmd.velocity.y, self.pos_cmd.velocity.x)
                    
                    # If we are not turning, what should the yaw angle be?
                    # If we are close to the goal point, hold a fixed yaw angle
                    x_gp_diff = self.goal_point.pose.position.x - self.vins_odom.pose.pose.position.x
                    y_gp_diff = self.goal_point.pose.position.y - self.vins_odom.pose.pose.position.y
                    if(math.sqrt(math.pow(x_gp_diff,2) + math.pow(y_gp_diff,2)) < self.gp_thresh):
                        self.gp_reached = True
                        if(self.gp_switch):
                            rospy.loginfo_throttle(2, 'Commanding trajectory, nearing current goal point')
                            self.end_psi_des = self.yaw
                            self.gp_switch = False
                    else:
                        self.gp_reached = False
                        self.gp_switch = True
                        # Use velocity vector instead
                        psi_des = math.atan2(self.pos_cmd.velocity.y, self.pos_cmd.velocity.x)
                    
                    if(self.pos_cmd.trajectory_flag == 3):
                        if(self.STATE_TRAJ_):
                            self.CMD_HOVER_ = True
                        rospy.loginfo_throttle(2, 'End of current trajectory')
                        self.initial_turn = True
                        if(self.end_traj_switch):
                            self.end_traj_switch = False
                            command_out.z = self.end_psi_des
                            command_out.x = self.pos_cmd.position.x
                            command_out.y = self.pos_cmd.position.y
                            #command_out.F = self.pos_cmd.position.z
                            command_out.F = self.height_des
                        command_out.x_vel = self.pos_cmd.velocity.x
                        command_out.y_vel = self.pos_cmd.velocity.y
                        command_out.z_vel = self.pos_cmd.velocity.z
                        command_out.x_acc = self.pos_cmd.acceleration.x
                        command_out.y_acc = self.pos_cmd.acceleration.y
                        command_out.z_acc = self.pos_cmd.acceleration.z
                        command_out.controller_select = 2
                    else: 
                        rospy.loginfo_throttle(2, 'Commanding trajectory')
                        command_out.x = self.pos_cmd.position.x
                        command_out.y = self.pos_cmd.position.y
                        #command_out.F = self.pos_cmd.position.z
                        command_out.F = self.height_des
                        # Use velocity vector instead
                        command_out.z = psi_des
                        command_out.x_vel = self.pos_cmd.velocity.x
                        command_out.y_vel = self.pos_cmd.velocity.y
                        command_out.z_vel = self.pos_cmd.velocity.z
                        command_out.x_acc = self.pos_cmd.acceleration.x
                        command_out.y_acc = self.pos_cmd.acceleration.y
                        command_out.z_acc = self.pos_cmd.acceleration.z
                        command_out.controller_select = 2
                    self.btraj_cmd_pub.publish(command_out)
            self.control_status_pub.publish(self.control_status)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hl_cmd_handler', anonymous=True, disable_signals=True)
    hl_cmd_node = hl_cmd_handler()
    hl_cmd_node.start()
