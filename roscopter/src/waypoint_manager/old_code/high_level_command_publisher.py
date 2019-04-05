#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from rosflight_msgs.msg import Command
from rosflight_msgs.msg import RCRaw
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Path

class hl_cmd_handler(object):
    def __init__(self):
        # Vars
        self.rc_msg = RCRaw()
        self.current_gp = PositionCommand()
        self.loop_rate = rospy.Rate(50)
        self.goalpoint = Path()
        self.cmd_pub = rospy.Publisher('high_level_command', Command, queue_size=10)
        rospy.Subscriber('rc_raw', RCRaw, self.rc_cb)
        rospy.Subscriber('position_cmd', PositionCommand, self.pos_cmd_cb)

        self.goalpoint_pub = rospy.Publisher('goal', PoseStamped, queue_size=10)
        self.switch = False

        self.traj_flag = 0;

        # This should be in vehicle NED frame
        self.hl_cmd = Command()
        self.hl_cmd.ignore = 0
        self.hl_cmd.mode = 4
        self.hl_cmd.x = 0.0
        self.hl_cmd.y = 1.0
        self.hl_cmd.z = 0.0
        self.hl_cmd.F = -0.5

        # This is in the vicon frame??
        self.pose = PoseStamped()
        self.pose.pose.position.x = 2.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = .75

        # set up the goalpoint as a pos command
        self.gp_cmd = Command()
        self.gp_cmd.ignore = 0
        self.gp_cmd.mode = 4
        self.gp_cmd.x = 1.0
        self.gp_cmd.y = 1.0
        self.gp_cmd.z = 0.0
        self.gp_cmd.F = -0.5

        #self.goalpoint.poses.append(pose)

        self.hl_cmd2 = Command()

    
    def rc_cb(self, data):
        #rospy.loginfo("%f", data.values[1])
        self.rc_msg = data
        #rospy.loginfo("%f", rc_msg.values[1])

    def pos_cmd_cb(self, data):
        self.current_gp = data
        #rospy.loginfo('pos_cb')
        self.traj_flag = self.current_gp.trajectory_flag
        #rospy.loginfo('%d', self.traj_flag)
        if (((self.current_gp.position.x > 3) or (self.current_gp.position.x < -2)) or 
            ((self.current_gp.position.y > 3) or (self.current_gp.position.y < -3)) or
            ((self.current_gp.position.z > 3) or (self.current_gp.position.z < -3))):
            self.hl_cmd2 = self.hl_cmd
        else:
            self.hl_cmd2.ignore = 0
            self.hl_cmd2.mode = 4
            self.hl_cmd2.x = self.current_gp.position.x
            self.hl_cmd2.y = -self.current_gp.position.y
            self.hl_cmd2.z = self.current_gp.yaw
            self.hl_cmd2.F = -self.current_gp.position.z


    def start(self):
        while not rospy.is_shutdown():
            # handle waypoint command mode
        #    if (self.traj_flag == 1):
        #        self.cmd_pub.publish(self.hl_cmd2)
        #    elif(self.traj_flag == 3):
        #        self.cmd_pub.publish(self.gp_cmd)
        #    else:
        #        self.cmd_pub.publish(self.hl_cmd)
            
            # Send the goal point
            #if((self.rc_msg.values[5]) > 1500 and (not self.switch)):
            #    self.goalpoint_pub.publish(self.pose)
            #    self.switch = True
            #    rospy.loginfo('published waypoint!')
        
            if((self.rc_msg.values[6]) > 1500):
                self.cmd_pub.publish(self.gp_cmd)
            else:
                self.cmd_pub.publish(self.hl_cmd)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hl_cmd_handler', anonymous=True)
    hl_cmd_node = hl_cmd_handler()
    hl_cmd_node.start()
