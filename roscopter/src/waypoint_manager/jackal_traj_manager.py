#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from rosflight_msgs.msg import Command, BtrajCommand, RCRaw, ControlStatus
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Path, Odometry
from z_state_estimator.msg import ZStateEst
from marble_common.msg import Estop


import tf
import math

class hl_cmd_handler(object):
    def __init__(self):

        # Define variables
        self.command_out = TwistStamped()
        self.command_out.twist.linear.x = 0.0
        self.command_out.twist.angular.z = 0.0

        self.kp_fwd = 1.0
        self.kd_fwd = .1

        self.kp_steer = 1.0
        self.kd_steer = .1

        self.tau = 0.05
        self.last_pos_err = 0.0

        self.max_vel = .5
        self.max_yawrate = .2

        # Define Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', TwistStamped, queue_size=10)

        # Define Subscribers
        rospy.Subscriber('position_cmd', PositionCommand, self.pos_cmd_cb)
        rospy.Subscriber('compass_pose', Odometry, self.state_est_cb)
        #rospy.Subscriber('estop', Estop, self.estop_cb)

    def pos_cmd_cb(self, msg):
        self.pos_cmd = msg

    def state_est_cb(self, msg):
        self.state_estimate = msg

    # GOAL POINT SWitCHING LOGIC NEEDS TO
    # BE VERIFIED
    #def goalpt_cb(self, msg):
    #    self.newest_frontier = msg
    #    # Update the goal point only when
    #    # the current traj has finished
    #    if(self.gp_reached):
    #        self.goal_point = msg
    #        self.gp_reached = False
    #    self.new_goal = True

    #    if(self.elapsed > self.time_limit):
    #        self.goal_point = origin
    #    rospy.loginfo('Goal point received')

    def start(self):
        
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.elapsed = (rospy.Time.now() - start_time).to_sec()
            dt = (self.last_time - rospy.Time.now()).to_sec()
            self.last_time = rospy.Time.now()

            # Check the status of Btraj Trajectory and create the cmd_vel message
            if(self.pos_cmd.trajectory_flag == 0):
                # We don't have a trajectory yet, stay in place
                rospy.loginfo_throttle(2,'No trajectory, staying in place')
                self.command_out.twist.linear.x = 0.0
                self.command_out.twist.angular.z = 0.0
            elif(self.pos_cmd.trajectory_flag == 1):
                # Now we have a trajectory, generate commands
                rospy.loginfo_throttle(2,'Have trajectory, following...')
               
                # Find error between current and desired states
                pos_err = sqrt(pow((self.pos_cmd.position.position.x - self.state_est.position.position.x),2) + pow((self.pos_cmd.position.position.y - self.state_est.position.position.y),2))

                # Find the proportional contribution
                p_term = self.kp_fwd*pos_err;

                # Find the derivative contribution
                if (self.kd_fwd > 0.0):
                    diff = (2 * self.tau - dt) / (2 * self.tau + dt) * self.last_diff + 2 / (2 * self.tau + dt) * (pos_err - self.last_pos_err)
                    d_term = self.kp_fwd * diff
                else:
                    d_term = 0.0

                self.last_diff = diff;
                self.last_pos_err = pos_err

                self.command_out.twist.linear.x = np.clip(p_term - d_term, 0.0, self.max_vel)

                # Compute the desired yaw angle based on desired velocity vector
                psi_des = math.atan2(self.pos_cmd.velocity.y, self.pos_cmd.velocity.x)

                # Find the yaw error term by differencing desired yaw angle and current
                yaw_err = psi_des - self.psi
            
                if (yaw_err > self.yaw_thresh):
                    self.command_out.twist.linear.x = 0.0
            
                yaw_p_term = kp_steer*yaw_err

                self.command_out.twist.angular.z = np.clip(yaw_p_term, -self.max_yawrate, self.max_yawrate
            
            elif(self.pos_cmd.trajectory_flag == 3):
                # We have reached the end of the current trajectory
                # Finish current route and wait for new trajectory

                rospy.loginfo_throttle(2,'End of current trajectory, staying in place')
                self.command_out.twist.linear.x = 0.0
                self.command_out.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(command_out)
            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node('hl_cmd_handler', anonymous=True, disable_signals=True)
    hl_cmd_node = hl_cmd_handler()
    hl_cmd_node.start()
