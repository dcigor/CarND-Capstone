#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

IS_IN_VEHICLE = False
MAX_SPEED_IN_VEHICLE = 4.4704 #[m/s] = 10mph  Max vehicle speed while the code is running 

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)
	self.controller = Controller(vehicle_mass, fuel_capacity, wheel_base, wheel_radius, steer_ratio, 0.1, max_lat_accel, max_steer_angle, brake_deadband)

        # TODO: Subscribe to all the topics you need to
	rospy.Subscriber('/current_velocity',TwistStamped, self.speed_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        self.dbw_enabled = False

	self.speed = 0
	self.target_long_speed = 0
	self.target_yaw_rate  = 0

	self.sample_rate = 5 #10Hz

	self.now = None
	self.prev_now = rospy.get_rostime()

        self.loop()

    def loop(self):
        rate = rospy.Rate(self.sample_rate) 
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
		
		self.now = rospy.get_rostime()	#Store current time
		dt = (self.now - self.prev_now).to_sec()	#Calculate delta with previous time
		self.prev_now = self.now	#Store current time for next iteartion

		throttle, brake, steering = self.controller.control(self.target_long_speed, self.target_yaw_rate, self.speed, self.dbw_enabled, dt)
		
        	self.publish(throttle, brake, steering)		

 		rate.sleep()


    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def dbw_enabled_cb(self, enabled):
        self.dbw_enabled = enabled.data

    def twist_cb(self, twist):
        if self.dbw_enabled == False:
            return
	
	#If the code is going to run in the vehicle, limit the target speed to "MAX_SPEED_IN_VEHICLE"
	if (IS_IN_VEHICLE ==True):
		self.target_long_speed = min(MAX_SPEED_IN_VEHICLE, twist.twist.linear.x) #In m/s
	else:
		self.target_long_speed = twist.twist.linear.x #In m/s


	self.target_yaw_rate = twist.twist.angular.z #In rad/s?

## rgpadin: this is calculated in the function loop()
        ## pass the throttle and steering info to the car
        #speed = twist.twist.linear.x
        #steer = twist.twist.angular.z
        #brake = 0.0
        #if speed < 0.01:
        #    brake = 100.0

        #self.publish(speed, brake, steer)

    def speed_cb(self, speed_msg):
	self.speed = speed_msg.twist.linear.x #in m/s

if __name__ == '__main__':
    DBWNode()
