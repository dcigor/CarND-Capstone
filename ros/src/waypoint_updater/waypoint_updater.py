#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import copy

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
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        #Add a subscriber for vehicle speed	
        rospy.Subscriber('/current_velocity',TwistStamped, self.speed_cb)

        # TODO: Add other member variables you need below
        self.waypoints   = []   # all waypoints delivered to us from /base_waypoints publisher
        self.current_pos = None # current position of the car, initially unset
        self.red_light_waypoint_index = -1

        self.prev_now = rospy.get_rostime()

        #publish /final_waypoints periodically
        self.publish(1)

        rospy.spin()

    def pose_cb(self, msg):
        self.current_pos = msg

    def waypoints_cb(self, lane):
        if len(lane.waypoints) == len(self.waypoints) and \
            lane.waypoints[ 0].pose.pose.position.x == self.waypoints[ 0].pose.pose.position.x and \
            lane.waypoints[-1].pose.pose.position.x == self.waypoints[-1].pose.pose.position.x:
            #we received exactly the same waypoints again: ignore
            return

        #remember new waypoints
        self.waypoints = lane.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.red_light_waypoint_index = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

	#Callback to get the vehicle speed
    def speed_cb(self, speed_msg):
		self.veh_speed = speed_msg.twist.linear.x #in m/s

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

        #step 1: jump by 100 points to find approximate range
        step       = 100
        curr_pos   = self.current_pos.pose.position
        best_index = -1
        best_dist  = 1000000
        for i in range(0, len(self.waypoints), step):
            dist = self.squared_distance(curr_pos, self.waypoints[i].pose.pose.position)
            if dist < best_dist:
                best_index = i
                best_dist  = dist

        #step 2: go by 1 when we get closer
        best_dist = 1000000
        start     = max(best_index-step, 0)
        end       = min(best_index+step, len(self.waypoints))
        for i in range(start, end):
            dist = self.squared_distance(curr_pos, self.waypoints[i].pose.pose.position)
            if dist < best_dist:
                best_index = i
                best_dist  = dist
        return best_index

    def is_simulator(self):
        #simulator has 10902 waypoints.
        #todo: find a better way to see if we are in simulator or not.
        return len(self.waypoints) == 10902

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

            set_speed = 9.0		#Set Speed [mph]
            #in the simulator allow speed to 50 mph.
            if self.is_simulator():
                set_speed = 40

            speed = set_speed*0.44704	#Set Speed converted to [m/s]
            max_acc = 5		#Maximum acceleration [m/s2]
            minSpeed = 5	#Minimum set speed for acceleration(rolling speed) [m]
            t_pred = 1.		#Length of the predictions, this prediction shall be equal or higher than the waypoint_updated rate [s]
            safeDist = 10.	#Distance before reaching the traffic light [m]
            min_dist = 0.2	#Minimum distance to calculate the target_speed [m]
            min_acc2brake = 1.0	#Minimum acceleration required to start to brake due to a traffic light [m/s2]
            approaching_speed = 2.5 #Set speed to approach to the traffic light stop position [m/s]

            # stop if red light is ahead

            #No traffic light detected, the vehicle shall follow the set speed selected
            if(self.red_light_waypoint_index == -1):
                pred_speed = self.veh_speed + 0.5*max_acc*t_pred*t_pred
                for w in range(0,len(lane.waypoints)):
                    dist = self.distance(lane.waypoints, 0, w)
                    vel = min(speed, math.sqrt(pred_speed*pred_speed + 2 * max_acc * dist))
                    vel = max(minSpeed, vel)
                    lane.waypoints[w].twist.twist.linear.x = vel

                #rospy.logerr("No TL	SetSpeed: %.1f", lane.waypoints[0].twist.twist.linear.x)

            #Traffic light detected
            else:
                #Calculate the acceleration required to stop within a safe distance to the traffic light
                dist2TL = self.distance(self.waypoints, best_index, best_index + self.red_light_waypoint_index)		
                dist2TL = dist2TL - self.veh_speed*t_pred

                aMin = speed*speed/(2.0*(dist2TL-safeDist))

                #The acceleration required is big enough to start to brake due to the red traffic light
                if(aMin > min_acc2brake):
                    #Calculate the appropiate speed for each waypoint to achieve the right speed at the last way point before the traffic light
                    pred_speed = self.veh_speed - 0.5*aMin*t_pred*t_pred
                    for w in range(0,len(lane.waypoints)):
                        dist = max(min_dist,self.distance(lane.waypoints, 0, w))
                        vel = min(speed, math.sqrt(pred_speed*pred_speed + 2 * max_acc * dist))
                        lane.waypoints[w].twist.twist.linear.x = max(approaching_speed,vel)
			
                    #rospy.logerr("Close TL	SetSpeed: %.1f		Dist: %.1f 	Acc: %0.2f", lane.waypoints[0].twist.twist.linear.x,dist2TL,aMin)

                    #When the vehicle passed the safety distance, the vehicle shall request a set speed =0 to stop the vehicle.
                elif(aMin<0):
                    for w in range(0,len(lane.waypoints)):
                        lane.waypoints[w].twist.twist.linear.x = 0.	#STOP required.

                    #rospy.logerr("Very Close TL	SetSpeed: %.1f		Dist: %.1f 	Acc: %0.2f", lane.waypoints[0].twist.twist.linear.x,dist2TL, aMin)

                #When the acceleration estimated is too low to start braking (the vehicle is too far away from the traffic light), the normal set speed shall be applied.
                else:
                    for w in lane.waypoints:
                        w.twist.twist.linear.x = speed

                    #rospy.logerr("TOO early TO BRAKE!! SetSpeed: %.1f		Dist: %.1f	Acc: %0.2f", lane.waypoints[0].twist.twist.linear.x,dist2TL,aMin)

            self.final_waypoints_pub.publish(lane)
            rate.sleep()

            #this is an example of how to measure time it takes for a piece of code to execute 
            #import timeit
            #rospy.logerr("new %f", timeit.Timer(lambda: self.index_of_nearest_point()).timeit(number=100))

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
