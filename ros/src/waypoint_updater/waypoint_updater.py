#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints   = []   # all waypoints delivered to us from /base_waypoints publisher
        self.current_pos = None # current position of the car, initially unset

        #publish /final_waypoints at 1 Hz
        self.publish(1)
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pos = msg
        pass

    def waypoints_cb(self, lane):
        # TODO: Implement
        if len(lane.waypoints) == len(self.waypoints) and \
            lane.waypoints[ 0].pose.pose.position.x == self.waypoints[ 0].pose.pose.position.x and \
            lane.waypoints[-1].pose.pose.position.x == self.waypoints[-1].pose.pose.position.x:
            #we received exactly the same waypoints again: ignore
            return

        #remember new waypoints
        self.waypoints = lane.waypoints

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

    def squared_distance(self, a, b):
        dx, dy, dz = a.x-b.x, a.y-b.y, a.z-b.z
        return dx*dx + dy*dy  + dz*dz

    # finds index of the nearest waypoint to the current point.
    # returns index or -1 if points or current position are not set
    def index_of_nearest_point(self):
        if self.current_pos is None:
            return -1

        curr_pos   = self.current_pos.pose.position
        best_index = -1
        best_dist  = 1000000
        for i in range(len(self.waypoints)):
            dist = self.squared_distance(curr_pos, self.waypoints[i].pose.pose.position)
            if dist < best_dist:
                best_index = i
                best_dist  = dist
        return best_index

    def publish(self, in_rate):
        rate = rospy.Rate(in_rate)
        while not rospy.is_shutdown():
            best_index = self.index_of_nearest_point()
            if best_index == -1:
                rate.sleep() # did not receive curr position or points yet
                continue

            # the nearest waypoint might behind us.
            # TODO: consider using direction from the Quarternion of the current position to
            # figure it out exactly.
            # for now instead: simply ignore the first point
            best_index += 1

            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)

            # use 200(LOOKAHEAD_WPS) of the existing points beginning at the best position
            lane.waypoints = self.waypoints[best_index : best_index+LOOKAHEAD_WPS]

            self.final_waypoints_pub.publish(lane)
            rate.sleep()

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
