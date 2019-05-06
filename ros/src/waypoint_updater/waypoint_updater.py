#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

	self.pose = None
	self.base_waypoints = None
	self.waypoints_2d = None
	self.waypoints_tree = None
	self.stopline = -1

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
	rospy.Subscriber('/traffic_waypoint', Int32, self.trafficpoints_cb)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
	self.loop()

    def loop(self):
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		if self.pose and self.base_waypoints:
			self.publish_waypoints()
		rate.sleep()

    def trafficpoints_cb(self, trafficpoints):
	self.stopline = trafficpoints.data

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
	if not self.waypoints_2d:
		self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
		self.waypoints_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        pass

    def obstacle_cb(self, msg):
        pass

    def get_closest_waypoint_idx(self):
	x = self.pose.pose.position.x
	y = self.pose.pose.position.y
	if not self.waypoints_tree:
		return 0
	closest_waypoint = self.waypoints_tree.query([x, y], 1)[1]
	closest_coord = self.waypoints_2d[closest_waypoint]
	prev_coord = self.waypoints_2d[closest_waypoint - 1]
	cl_vect = np.array(closest_coord)
	prev_vect = np.array(prev_coord)
	pos_vect = np.array([x,y])
	val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
	if val > 0:
		closest_waypoint = (closest_waypoint + 1) % len(self.waypoints_2d)
	return closest_waypoint

    def publish_waypoints(self):
	final_lane = self.get_final_lane()
	self.final_waypoints_pub.publish(final_lane)

    def get_final_lane(self):
	lane = Lane()
	closest_waypoint = self.get_closest_waypoint_idx()
	ref_waypoints = self.base_waypoints.waypoints[closest_waypoint : closest_waypoint + LOOKAHEAD_WPS]
	is_stopline_near = self.stopline != -1 and self.stopline < closest_waypoint + LOOKAHEAD_WPS
	if is_stopline_near:
		lane.waypoints = self.decelerate(ref_waypoints, closest_waypoint)
	else:
		lane.waypoints = ref_waypoints
	return lane

    def decelerate(self, ref_waypoints, closest_waypoint):
	new_waypoints = []
	stop_idx = max(self.stopline - closest_waypoint - 7, 0) 
	for i, wp in enumerate(ref_waypoints):
		p = Waypoint()
		p.pose = wp.pose
		
		dist = self.distance(ref_waypoints,i, stop_idx)
		vel = math.sqrt(2* 0.5 * dist)
		if vel < 1.:
			vel = 0.
		p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
		new_waypoints.append(p)
	return new_waypoints

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
