#!/usr/bin/env python

import numpy as np
import rospy, tf
from geometry_msgs.msg import PoseStamped, Quaternion
from rosflight_msgs.msg import Command, BtrajCommand, RCRaw, VehicleStatus
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Path, Odometry
from z_state_estimator.msg import ZStateEst
import tf
import math

class hl_cmd_handler(object):
    def __init__(self):
        
        # Get Waypoints for Automatic Take Off
        try:
            self.waypoint_list = rospy.get_param('/waypoint_manager/waypoints')
        except KeyError:
            rospy.logfatal('waypoints not set')
        self.threshold = rospy.get_param('~threshold', 0.1)
        
                
        # Vars
        self.rc_msg = RCRaw()
        self.pos_cmd = PositionCommand()
        self.vehicle_status = VehicleStatus()
        self.loop_rate = rospy.Rate(20)
        
        # Define Publishers
        self.cmd_pub = rospy.Publisher('high_level_command', Command, queue_size=10)
        self.btraj_cmd_pub = rospy.Publisher('btraj_command', BtrajCommand, queue_size=10)
        self.btraj_goalpt_pub = rospy.Publisher('goal', PoseStamped, queue_size=10)
        self.vehicle_status_pub = rospy.Publisher('vehicle_status', VehicleStatus, queue_size=10)

        # Define Subscribers
        rospy.Subscriber('goal', PoseStamped, self.goalpt_cb)
        rospy.Subscriber('rc_raw', RCRaw, self.rc_cb)
        rospy.Subscriber('position_cmd', PositionCommand, self.pos_cmd_cb)
        rospy.Subscriber('vins_estimator/odometry', Odometry, self.vins_odom_cb)
        rospy.Subscriber('z_state_estimator/z_state_estimate', ZStateEst, self.z_state_cb)
        
        self.vehicle_status = VehicleStatus()
        self.vehicle_status.control_status = 0
        self.vehicle_status.replan = 0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.end_yaw = 0.0

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

        self.takeoff_cmd = BtrajCommand()
        self.takeoff_cmd.ignore = 0
        self.takeoff_cmd.mode = 4
        self.takeoff_cmd.controller_select = 1

        self.home_cmd = BtrajCommand()
        self.home_cmd.ignore = 0
        self.home_cmd.mode = 4
        self.home_cmd.controller_select = 1
        self.home_cmd.x = 0.0
        self.home_cmd.y = 0.0
        self.home_cmd.z = 0.0
        self.home_cmd.F = 0.55

        self.man_cmd = BtrajCommand()
        self.man_cmd.ignore = 7
        self.man_cmd.mode = 4
        self.man_cmd.controller_select = 1
        self.man_cmd.x = 0.0
        self.man_cmd.y = 0.0
        self.man_cmd.z = 0.0
        self.man_cmd.F = 0.55
        
        self.turn_mnvr = BtrajCommand()
        self.turn_mnvr.ignore = 0
        self.turn_mnvr.mode = 4
        self.turn_mnvr.controller_select = 2
        self.turn_mnvr.x = 0.0
        self.turn_mnvr.y = 0.0
        self.turn_mnvr.z = 0.0
        self.turn_mnvr.F = 0.75
        self.turn_switch = True
        self.turn_maneuver = False

    def rc_cb(self, msg):
        self.rc_msg = msg
        #rospy.loginfo("6: %d, 7: %d, 8: %d", self.rc_msg.values[5], self.rc_msg.values[6],self.rc_msg.values[7])
    
    def z_state_cb(self, msg):
        self.z_state_msg = msg

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
#        command_out  = BtrajCommand()
#        command_out.ignore = 0
#        command_out.mode = 4
#        command_out.controller_select = 2

        # Takeoff initialization
        self.take_off = 1
        self.current_waypoint_index = 0
        current_waypoint = np.array(self.waypoint_list[0])
        self.takeoff_cmd.x = current_waypoint[0]
        self.takeoff_cmd.y = current_waypoint[1]
        self.takeoff_cmd.z = 0
        self.takeoff_cmd.F = current_waypoint[2]
        self.takeoff_cmd.x_vel = current_waypoint[3]
        self.takeoff_cmd.y_vel = current_waypoint[4]
        self.takeoff_cmd.z_vel = current_waypoint[5]
        current_position = np.zeros(3)
        error = 100000000000000000

        while not rospy.is_shutdown():

            if(self.take_off == 1):
                current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
                (r, p, y) = tf.transformations.euler_from_quaternion([self.vins_odom.pose.pose.orientation.x, self.vins_odom.pose.pose.orientation.y, self.vins_odom.pose.pose.orientation.z, self.vins_odom.pose.pose.orientation.w])
                current_position = np.array([self.vins_odom.pose.pose.position.x,
                                             self.vins_odom.pose.pose.position.y,
                                             self.vins_odom.pose.pose.position.z])
                error = np.linalg.norm(-current_position[2] - current_waypoint[2])

                if error < self.threshold:
                    # Get new waypoint index
                    self.current_waypoint_index += 1
                    next_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
                    self.takeoff_cmd.x = next_waypoint[0]
                    self.takeoff_cmd.y = next_waypoint[1]
                    self.takeoff_cmd.z = 0
                    self.takeoff_cmd.F = next_waypoint[2]
                    self.takeoff_cmd.x_vel = next_waypoint[3]
                    self.takeoff_cmd.y_vel = next_waypoint[4]
                    self.takeoff_cmd.z_vel = next_waypoint[5]

                    if(self.current_waypoint_index == np.shape(self.waypoint_list)[0] - 1):
                        self.take_off = 0

                rospy.loginfo_throttle(2, 'Commanding home position')
                command_out = self.takeoff_cmd
                command_out.header.stamp = rospy.Time.now()
                self.btraj_cmd_pub.publish(command_out)

                rospy.loginfo_throttle(1, "current_waypoint z: ")
                rospy.loginfo_throttle(1, current_waypoint[3])
                rospy.loginfo_throttle(1, "vins_odom z: ")
                rospy.loginfo_throttle(1, self.vins_odom.pose.pose.position.z)
                rospy.loginfo_throttle(1, "error: ")
                rospy.loginfo_throttle(1, error)
                rospy.loginfo_throttle(1, "waypoint_index: ")
                rospy.loginfo_throttle(1, self.current_waypoint_index)
                rospy.loginfo_throttle(1, "take_off flag: ")
                rospy.loginfo_throttle(1, self.take_off)

            else:
                #self.checkControlState()

                # Trajectory Flag = 0 ==> no trajectory available
                if(self.rc_msg.values[6] < 1500):
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

                    # Handle Yaw state:
                    # We want to point at the commanded X, Y position state, 
                    # which should remain in front of the vehicle.

                    # Compute the relative yaw angle between
                    # current position and desired position
                    #x_pos_diff = self.pos_cmd.position.x - self.vins_odom.pose.pose.position.x
                    #y_pos_diff = self.pos_cmd.position.y - self.vins_odom.pose.pose.position.y

                    #psi_des = math.atan2(y_pos_diff, x_pos_diff)

                    # Use velocity vector instead
                    psi_des = math.atan2(self.pos_cmd.velocity.y, self.pos_cmd.velocity.x)
                    
                    #if the difference between the desired yaw angle and the current
                    # yaw angle is greater than pi/4, execute a turn only maneuver
                    if (not self.gp_reached and math.fabs((psi_des - self.yaw)) > .8):
                        rospy.loginfo_throttle(2, 'Executing turn maneuver')
                        self.turn_maneuver = True
                        self.turn_mnvr.header.stamp = rospy.Time.now()
                        if(self.turn_switch):
                            self.turn_mnvr.x = self.vins_odom.pose.pose.position.x
                            self.turn_mnvr.y = self.vins_odom.pose.pose.position.y
                            self.turn_mnvr.F = self.vins_odom.pose.pose.position.z
                            self.turn_switch = False
                        self.turn_mnvr.z = self.end_yaw
                        self.btraj_cmd_pub.publish(self.turn_mnvr)
                    # Else, execute the trajectory normally
                    else:
                        # If we are exiting a turn manuever, replan to reset the trajectory
                        if(self.turn_maneuver == True):
                            # We need to replan
                            rospy.loginfo("end turn")
                            self.vehicle_status.replan = 1
                            self.turn_maneuver = False
                        else:
                            self.vehicle_status.replan = 0

                        self.turn_switch = True
                        x_gp_diff = self.goal_point.pose.position.x - self.vins_odom.pose.pose.position.x
                        y_gp_diff = self.goal_point.pose.position.y - self.vins_odom.pose.pose.position.y
                        
                        # If we are not turning, what should the yaw angle be?
                        # If we are close to the goal point, hold a fixed yaw angle
                        x_gp_diff = self.goal_point.pose.position.x - self.vins_odom.pose.pose.position.x
                        y_gp_diff = self.goal_point.pose.position.y - self.vins_odom.pose.pose.position.y
                        if(math.sqrt(math.pow(x_gp_diff,2) + math.pow(y_gp_diff,2)) < self.gp_thresh):
                            self.gp_reached = True
                            if(self.gp_switch):
                                rospy.loginfo_throttle(2, 'Commanding trajectory, nearing current goal point')
                                psi_des = math.atan2(self.pos_cmd.velocity.y, self.pos_cmd.velocity.x)
                                self.gp_switch = False
                        else:
                            self.gp_reached = False
                            self.gp_switch = True
                            # Use velocity vector instead
                            psi_des = math.atan2(self.pos_cmd.velocity.y, self.pos_cmd.velocity.x)
                        
                        if(self.pos_cmd.trajectory_flag == 3):
                            rospy.loginfo_throttle(2, 'End of current trajectory')
                            if(self.end_traj_switch):
                                self.end_traj_switch = False
                                command_out.z = self.end_yaw
                                command_out.x = self.pos_cmd.position.x
                                command_out.y = self.pos_cmd.position.y
                                command_out.F = self.pos_cmd.position.z
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
                            command_out.F = self.pos_cmd.position.z
                            # Use velocity vector instead
                            command_out.z = psi_des;
                            command_out.x_vel = self.pos_cmd.velocity.x
                            command_out.y_vel = self.pos_cmd.velocity.y
                            command_out.z_vel = self.pos_cmd.velocity.z
                            command_out.x_acc = self.pos_cmd.acceleration.x
                            command_out.y_acc = self.pos_cmd.acceleration.y
                            command_out.z_acc = self.pos_cmd.acceleration.z
                            command_out.controller_select = 2
                        self.btraj_cmd_pub.publish(command_out)
                self.vehicle_status_pub.publish(self.vehicle_status)
                self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hl_cmd_handler', anonymous=True, disable_signals=True)
    hl_cmd_node = hl_cmd_handler()
    hl_cmd_node.start()
