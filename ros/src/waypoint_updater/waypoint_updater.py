#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

import numpy as np
from scipy.spatial import KDTree

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None

        self.loop()

    # main loop function
    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                # Get closest waypoints
                closest_waypoint_index = self.get_closest_waypoint_index()
                self.publish_waypoints(closest_waypoint_index)
            rate.sleep()

        pass

    # callback functions
    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # called one time only because the base waypoins never changed, and it is ineffecient to send constant data periodically
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    # getter functions
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def get_closest_waypoint_index(self):
        # x, y car coordinates
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        # select one closest position, and get only its index
        if not self.waypoints_tree:
            return 0

        closest_index = self.waypoints_tree.query([x, y], 1)[1]

        # check if the closest coordinate is front or behind the car
        closest_coordinate = self.waypoints_2d[closest_index]
        prev_coordinate = self.waypoints_2d[closest_index - 1]

        # equation for hyperplane throught he closest coordinate
        closest_vector = np.array(closest_coordinate)
        prev_vector = np.array(prev_coordinate)
        pose_vector = np.array([x, y])

        # check if the dot product positive or negative
        direction = np.dot(closest_vector - prev_vector, pose_vector - closest_vector)

        # if the closest coordinate behind the car, the direction variable from the dot product is positive 
        # because it point on the same direction, then ignore it, take the next one
        if direction > 0:
            closest_index = (closest_index + 1) % len(self.waypoints_2d)
        return closest_index


    # setter functions
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    # utilities functions
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def publish_waypoints(self, closest_waypoint_index):
        # the message type need to be a lane
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_waypoint_index:closest_waypoint_index + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
