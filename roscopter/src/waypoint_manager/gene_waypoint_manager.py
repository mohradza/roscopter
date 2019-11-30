#!/usr/bin/env python

import numpy as np

import rospy, tf

from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile


class WaypointManager():

    def __init__(self):

	# set parameters
#	rospy.set_param('waypoints', [[0.0, 0.0, -2.0, 0.0],[4.0, 0.0, -2.0, 0.0],[4.0, 1.0, -2.0, 0.0], [0.0, 1.0, -2.0, 0.0], [0.0, 0.0, -2.0, 0.0], [1.0, 1.0, -2.0, 0.0], [2.0, 0.0, -2.0, 0.0], [1.0, -1.0, -2.0, 0,0], [0.0, 0.0, -2.0, 0.0], [0.0, 0.0, 0.0, 0.0]])
        #rospy.get_param('waypoint_manager/waypoints')

        # get parameters
        try:
            self.waypoint_list = rospy.get_param('/waypoint_manager/waypoints')
            rospy.loginfo(self.waypoint_list)
	except KeyError:
            rospy.logfatal('waypoints not set')
            rospy.signal_shutdown('Parameters not set')

        # how close does the MAV need to get before going to the next waypoint?
        self.threshold = rospy.get_param('waypoint_manager/threshold', 0.5) # 5
	self.cyclical_path = rospy.get_param('waypoint_manager/cycle', False) # True


        self.prev_time = rospy.Time.now()
	# Set up Last Position Variable (used for yaw wrapping)
	self.last_position = 0
	global error_x
	global error_y
	global error_F
	global error_z
	error_x = 0
	error_y = 0
	error_F = 0
	error_z = 0

	# set up Services
        self.add_waypoint_service = rospy.Service('add_waypoint', AddWaypoint, self.addWaypointCallback)
        self.remove_waypoint_service = rospy.Service('remove_waypoint', RemoveWaypoint, self.addWaypointCallback)
        self.set_waypoint_from_file_service = rospy.Service('set_waypoints_from_file', SetWaypointsFromFile, self.addWaypointCallback)

        # Set Up Publishers and Subscribers
	rospy.loginfo("Set Up Publishers and Subscribers")
        self.xhat_sub_ = rospy.Subscriber('odom', Odometry, self.odometryCallback, queue_size=5)
        self.waypoint_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)

        self.current_waypoint_index = 0

        command_msg = Command()
        
        current_waypoint = np.array(self.waypoint_list[0])

        command_msg.x = current_waypoint[0]
        command_msg.y = current_waypoint[1]
        command_msg.F = current_waypoint[2]
        if len(current_waypoint) > 3:
            command_msg.z = current_waypoint[3]
        else:
            next_point = self.waypoint_list[(self.current_waypoint_index + 1) % len(self.waypoint_list)]
            delta = next_point - current_waypoint
            command_msg.z = np.atan2(delta[1], delta[0])
        command_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
        self.waypoint_pub_.publish(command_msg)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def addWaypointCallback(req):
        print("addwaypoints")

    def removeWaypointCallback(req):
        print("remove Waypoints")

    def setWaypointsFromFile(req):
        print("set Waypoints from File")

    def odometryCallback(self, msg):
        # Get error between waypoint and current state
        current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        current_position = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     msg.pose.pose.position.z,
                                     -y])
	#if current_waypoint[3] > y + 0.95*6.283:
	#	current_waypoint[3] = current_waypoint[3] - 6.283
	#	rospy.loginfo("Subtracted 2 * pi from Yaw Waypoint")
	#if current_waypoint[3] < y - 0.95*6.283:
	#	current_waypoint[3] = current_waypoint[3] + 6.283 
	#	rospy.loginfo("Added 2 * pi to Yaw Waypoint")
        
	#if current_position[3] > self.last_position + 0.95*6.283:
        #       current_position[3] = current_position[3] - 6.283
        #       rospy.loginfo("Subtracted 2 * pi from Yaw")
        #if current_position[3] < self.last_position - 0.95*6.283:
        #       current_position[3] = current_position[3] + 6.283 
        #       rospy.loginfo("Added 2 * pi to Yaw")
	rospy.loginfo_throttle(1, current_waypoint)
	rospy.loginfo_throttle(1, current_position)
        error_x = current_waypoint[0] - current_position[0]
        error_y = current_waypoint[1] - current_position[1]
        error_F = current_waypoint[2] - current_position[2]
        error_z = current_waypoint[3] - current_position[3]
	if error_z >  0.95 * 6.283:
		error_z = error_z - 6.283
        if error_z < -0.95 * 6.283:
                error_z = error_z + 6.283

        #error = np.linalg.norm(current_position - current_waypoint)
        rospy.loginfo_throttle(1, error_x)
        rospy.loginfo_throttle(1, error_y)
        rospy.loginfo_throttle(1, error_F)
	rospy.loginfo_throttle(1, error_z)

	self.last_position = current_position[3]

        #if error < self.threshold:
	if -0.2 < error_x < 0.2 and -0.2 < error_y < 0.2 and -0.2 < error_F < 0.2 and -0.2 < error_z < 0.2:
		rospy.loginfo("move to next waypoint")
		# Get new waypoint index
		self.current_waypoint_index += 1
		if self.cyclical_path:
			self.current_waypoint_index %= len(self.waypoint_list)
		else:
			if self.current_waypoint_index > len(self.waypoint_list):
				self.current_waypoint_index -=1
		next_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
		command_msg = Command()
		rospy.loginfo(command_msg)
		command_msg.x = next_waypoint[0]
		command_msg.y = next_waypoint[1]
		command_msg.F = next_waypoint[2]
		rospy.loginfo(command_msg)
		if len(current_waypoint) > 3:
			command_msg.z = next_waypoint[3]
		else:
			next_point = self.waypoint_list[(self.current_waypoint_index + 1) % len(self.waypoint_list)]
			delta = next_point - current_waypoint
			command_msg.z = np.atan2(delta[1], delta[0])
		command_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
		self.waypoint_pub_.publish(command_msg)

if __name__ == '__main__':
    rospy.init_node('waypoint_manager', anonymous=True)
    try:
        wp_manager = WaypointManager()
    except:
		rospy.ROSInterruptException
    pass
