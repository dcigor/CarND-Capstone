#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np
import cv2

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        
        #if this is uncommented, the car fails to move in the simulator.
        #TODO: investigate. maybe too much data ?
        #sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        #sub3  = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6   = rospy.Subscriber('/image_color', Image, self.image_cb)
        sub_fw = rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config   = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.final_waypoints = Lane()

        rospy.spin()

    def final_waypoints_cb(self, lane):
        self.final_waypoints = lane

    def pose_cb(self, msg):
        self.pose = msg

    def squared_dist(self, dx, dy):
        return dx*dx+dy*dy

    #find nearest light
    def find_nearest_light(self):
        light_positions = self.config['stop_line_positions']
        waypoints       = self.final_waypoints.waypoints
        if len(light_positions) == 0 or len(waypoints) < 2:
            return []

        x0 = waypoints[0].pose.pose.position.x
        y0 = waypoints[0].pose.pose.position.y
        x1 = waypoints[1].pose.pose.position.x
        y1 = waypoints[1].pose.pose.position.y
        best_dist  = 1000000
        best_light = []
        for light in light_positions:
            dist = self.squared_dist(light[0]-x1, light[1]-y1)
            if dist < best_dist:
                best_dist  = dist
                best_light = light

        # if the light is too far or behind us: ignore it
        if best_dist > 10000 or self.squared_dist(best_light[0]-x0, best_light[1]-y0) < best_dist:
            return []

        return best_light

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        rospy.logerr("TLD WAYPOINTS")

    def traffic_cb(self, msg):
        self.lights = msg.lights
        rospy.logerr("TLD TRAFFIC")

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image    = True
        self.camera_image = msg
        light_wp, state   = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        return 0


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image

        x = 0
        y = 0

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = self.find_nearest_light()
        if len(light) == 0:
            return -1, TrafficLight.UNKNOWN

        color, pixel_count = self.get_light_color()
        if color == TrafficLight.UNKNOWN:
            return -1, TrafficLight.UNKNOWN

        wp_index = self.nearest_waypoint(light[0], light[1])
        return wp_index, color

        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)

        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

    #currently: detect red color only, as the car can proceed otherwise as if it there was no light.
    #TODO: detect green and yellow lights as well
    def get_light_color(self):
        #this is an image above the road, exactly where the light should be
        img = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #filter red pixels in HLS color space
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        hue = hls[:,:,0]
        lum = hls[:,:,1]
        sat = hls[:,:,2]

        red = np.zeros_like(hue)
        #the hue of red color wraps: it either a small or a large value.
        red[((hue < 20) | (hue>150)) & (lum>50) & (sat>150)] = 255

        #use erosion to eliminate small noise
        kernel = np.ones((5,5),np.uint8)
        eroded = cv2.erode(red, kernel, iterations = 1)

        #how many pixels are red?
        res = np.count_nonzero(red)
        if res > 100: # assume red pixels only belong to the traffic light
            return TrafficLight.RED, res

        return TrafficLight.UNKNOWN, res

    def nearest_waypoint(self, x, y):
        best_index = 0
        best_dist  = 1000000
        waypoints  = self.final_waypoints.waypoints
        for i in range(len(waypoints)):
            pos  = waypoints[i].pose.pose.position
            dist = self.squared_dist(x-pos.x, y-pos.y)
            if dist < best_dist:
                best_index = i
                best_dist  = dist
        return best_index

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
