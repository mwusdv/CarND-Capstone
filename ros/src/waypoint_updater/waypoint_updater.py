#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32, Float32

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5
PUBLISHING_RATE = 20

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')


        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=0)
        self.turning_angle_pub = rospy.Publisher('turning_angle', Float32, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_lane = None
        self.waypoints_2d = None
        self.pose = None
        self.last_pose = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
        self.closest_idx = -1
        

        self.loop()

    def loop(self):
        rate = rospy.Rate(PUBLISHING_RATE)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # check if the closest is ahead of behind the vehicle
        closest_waypoint = np.array(self.waypoints_2d[closest_idx])
        prev_waypoint = np.array(self.waypoints_2d[closest_idx-1])
        pose = np.array([x, y])

        val = np.dot(closest_waypoint - prev_waypoint, pose - closest_waypoint)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        
        min_turning_angle = 0
        if self.last_pose is not None:
            last_step = pose - self.last_pose
            last_norm = np.linalg.norm(last_step)
            if last_norm < 1e-6:
                self.closest_idx = closest_idx
                return closest_idx, min_turning_angle

            min_turning_angle = 100
            turning_angle = 100
            count = 0

            while turning_angle > 5 and count < 20:
                closest_waypoint = np.array(self.waypoints_2d[closest_idx])
                wx = closest_waypoint[0]
                wy = closest_waypoint[1]
                cur_step = closest_waypoint - pose
                
                val = np.dot(cur_step, last_step) / np.linalg.norm(cur_step) / last_norm
                turning_angle = np.arccos(val) * 180/np.pi
                rospy.logwarn('try_turning_angle: {}, car_x={}, car_y={}, w_x={}, w_y={}, count={}'.format(min_turning_angle, x, y,
                               wx, wy, count))
                if turning_angle < min_turning_angle:
                    min_turning_angle = turning_angle
                    opt_idx = closest_idx

                count += 1
                closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
            
            if turning_angle > 5:
                closest_idx = opt_idx
            closest_waypoint = np.array(self.waypoints_2d[closest_idx])
            wx = closest_waypoint[0]
            wy = closest_waypoint[1]
            rospy.logwarn('turning_angle: {}, car_x={}, car_y={}, w_x={}, w_y={}, count={}'.format(min_turning_angle, x, y,
                          wx, wy, count))
            rospy.logwarn('----------------------------------')

        self.closest_idx = closest_idx
        return closest_idx, min_turning_angle

    def publish_waypoints(self):
        final_lane, turning_angle = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
        self.turning_angle_pub.publish(turning_angle)

    def generate_lane(self):
        lane = Lane()

        closest_idx, turning_angle = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(waypoints, closest_idx)

        return lane, turning_angle

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            
            # Two waypints back from line so the front of
            # the car stops at the line
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) 
            
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.0:
                vel = 0.0

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp

    def pose_cb(self, msg):
        if self.pose is not None:
            self.last_pose = np.array([self.pose.pose.position.x, self.pose.pose.position.y])
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] 
                                  for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

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
