#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

       

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self._final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self._base_waypoints = None
        self._waypoints_2d = None
        self._pose = None
        self._waypoint_tree = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self._pose and self._base_waypoints:
                closest_waypoint_idx = self.get_closet_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()
    
    def get_closet_waypoint_idx(self):
        x = self._pose.pose.position.x
        y = self._pose.pose.position.y
        closet_idx = self._waypoint_tree.query([x, y], 1)[1]

        # check if the closet is ahead of behind the vehicle
        closet_waypoint = np.array(self._waypoints_2d[closet_idx])
        prev_waypoint = np.array(self._waypoints_2d[closet_idx-1])
        pos = np.array([x, y])

        val = np.dot(closet_waypoint - prev_waypoint, pos - closet_waypoint)
        if val > 0:
            closet_idx = (closet_idx + 1) % len(self._waypoints_2d)
        
        return closet_idx

    def publish_waypoints(self, closet_idx):
        lane = Lane()
        lane.header = self._base_waypoints.header
        lane.waypoints = self._base_waypoints.waypoints[closet_idx : closet_idx + LOOKAHEAD_WPS]
        self._final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self._pose = msg

    def waypoints_cb(self, waypoints):
        self._base_waypoints = waypoints
        if not self._waypoints_2d:
            self._waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] 
                                  for waypoint in waypoints.waypoints]
            self._waypoint_tree = KDTree(self._waypoints_2d)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

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
