#!/usr/bin/python3

"""
This is the main entry point for the particle filter exercise node. It
subscribes to laser, map, and odometry and creates an instance of
pf.PFLocaliser() to do the localisation.
"""

import rospy
import pf_localisation.pf
from pf_localisation.util import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped,
                                PoseArray, Quaternion )
from tf.msg import tfMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import pf_localisation
from threading import Lock

import sys
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np

class ParticleFilterLocalisationNode(object):
    def __init__(self):
        # ----- Minimum change (m/radians) before publishing new particle cloud and pose
        self._PUBLISH_DELTA = rospy.get_param("publish_delta", 0.1)  
        
        self._particle_filter = pf_localisation.pf.PFLocaliser()

        self._latest_scan = None
        self._last_published_pose = None
        self._initial_pose_received = False

        self._pose_publisher = rospy.Publisher("/estimatedpose", PoseStamped)
        self._amcl_pose_publisher = rospy.Publisher("/amcl_pose",
                                                    PoseWithCovarianceStamped)
        self._cloud_publisher = rospy.Publisher("/particlecloud", PoseArray, queue_size=10)
        self._tf_publisher = rospy.Publisher("/tf", tfMessage)

        rospy.loginfo("Waiting for a map...")
        try:
            ocuccupancy_map = rospy.wait_for_message("map", OccupancyGrid, 20)
        except:
            rospy.logerr("Problem getting a map. Check that you have a map_server"
                     " running: rosrun map_server map_server <mapname> " )
            sys.exit(1)
        rospy.loginfo("Map received. %d X %d, %f px/m." %
                      (ocuccupancy_map.info.width, ocuccupancy_map.info.height,
                       ocuccupancy_map.info.resolution))
        self._particle_filter.set_map(ocuccupancy_map)
        
        self._laser_subscriber = rospy.Subscriber("/robot_0/base_scan", LaserScan,
                                                  self._laser_callback,
                                                  queue_size=1)
        self._initial_pose_subscriber = rospy.Subscriber("/robot_0/initialpose",
                                                         PoseWithCovarianceStamped,
                                                         self._initial_pose_callback)
        self._odometry_subscriber = rospy.Subscriber("/robot_0/odom", Odometry,
                                                     self._odometry_callback,
                                                     queue_size=1)

        self._seeker_camera_subscriber = rospy.Subscriber('/robot_0/image',Image, self._robot_detection, queue_size=1)
    def _initial_pose_callback(self, pose):
        """ called when RViz sends a user supplied initial pose estimate """
        self._particle_filter.set_initial_pose(pose)
        self._last_published_pose = deepcopy(self._particle_filter.estimatedpose)
        self._initial_pose_received = True
        self._cloud_publisher.publish(self._particle_filter.particlecloud)

    def _odometry_callback(self, odometry):
        """
        Odometry received. If the filter is initialised then execute
        a filter predict step with odeometry followed by an update step using
        the latest laser.
        """
        if self._initial_pose_received:
            t_odom = self._particle_filter.predict_from_odometry(odometry)
            t_filter = self._particle_filter.update_filter(self._latest_scan)
            if t_odom + t_filter > 0.1:
                rospy.logwarn("Filter cycle overran timeslot")
                rospy.loginfo("Odometry update: %fs"%t_odom)
                rospy.loginfo("Particle update: %fs"%t_filter)

    def _robot_detection(self, ros_data):
        #np_arr = np.fromstring(ros_data.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        rgb_image = CvBridge().imgmsg_to_cv2(ros_data, desired_encoding="passthrough")
        ##image_np = cv2.cvtColor(np_arr, cv2.COLOR_GRAY2BGR)
        ##image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # load image
        ##img = cv2.imread(image_np)
        # Convert to HSV

        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        cv2.imwrite('Test_gray.jpg', rgb_image)

        # define range wanted color in HSV
        lower_val_red = np.array([0, 102, 0])
        upper_val_red = np.array([52, 255, 255])

        lower_val_purple = np.array([0, 188, 0])
        upper_val_purple = np.array([156, 242, 255])

        lower_val_green = np.array([52, 62, 0])
        upper_val_green = np.array([110, 255, 255])

        lower_val_blue = np.array([111, 50, 0])
        upper_val_blue = np.array([130, 255, 255])

        # Threshold the HSV image - any green color will show up as white
        mask_red = cv2.inRange(hsv, lower_val_red, upper_val_red)
        mask_purple = cv2.inRange(hsv, lower_val_purple, upper_val_purple)
        mask_green = cv2.inRange(hsv, lower_val_green, upper_val_green)
        mask_blue = cv2.inRange(hsv, lower_val_blue, upper_val_blue)
        # if there are any white pixels on mask, sum will be > 0
        hasred = np.sum(mask_red)
        haspurple = np.sum(mask_purple)
        hasgreen = np.sum(mask_green)
        hasblue = np.sum(mask_blue)
        if hasred > 0:
            print('Red detected!')

        if haspurple > 0:
            print('Purple detected!')

        if hasgreen > 0:
            print('Green detected!')

        if hasblue > 0:
            print('Blue detected!')

    def _laser_callback(self, scan):
        """
        Laser received. Store a ref to the latest scan. If robot has moved
        much, republish the latest pose to update RViz
        """
        self._latest_scan = scan
        if self._initial_pose_received:
            if  self._sufficientMovementDetected(self._particle_filter.estimatedpose):
                # ----- Publish the new pose
                self._amcl_pose_publisher.publish(self._particle_filter.estimatedpose)
                estimatedpose =  PoseStamped()
                estimatedpose.pose = self._particle_filter.estimatedpose.pose.pose
                estimatedpose.header.frame_id = "map"
                self._pose_publisher.publish(estimatedpose)
                
                # ----- Update record of previously-published pose
                self._last_published_pose = deepcopy(self._particle_filter.estimatedpose)
        
                # ----- Get updated particle cloud and publish it
                self._cloud_publisher.publish(self._particle_filter.particlecloud)
        
                # ----- Get updated transform and publish it
                self._tf_publisher.publish(self._particle_filter.tf_message)
    
    def _sufficientMovementDetected(self, latest_pose):
        """
        Compares the last published pose to the current pose. Returns true
        if movement is more the self._PUBLISH_DELTA
        """
        # ----- Check that minimum required amount of movement has occurred before re-publishing
        latest_x = latest_pose.pose.pose.position.x
        latest_y = latest_pose.pose.pose.position.y
        prev_x = self._last_published_pose.pose.pose.position.x
        prev_y = self._last_published_pose.pose.pose.position.y
        location_delta = abs(latest_x - prev_x) + abs(latest_y - prev_y)

        # ----- Also check for difference in orientation: Take a zero-quaternion,
        # ----- rotate forward by latest_rot, and rotate back by prev_rot, to get difference)
        latest_rot = latest_pose.pose.pose.orientation
        prev_rot = self._last_published_pose.pose.pose.orientation

        q = rotateQuaternion(Quaternion(w=1.0),
                             getHeading(latest_rot))   # Rotate forward
        q = rotateQuaternion(q, -getHeading(prev_rot)) # Rotate backward
        heading_delta = abs(getHeading(q))
        #rospy.loginfo("Moved by %f"%location_delta)
        return (location_delta > self._PUBLISH_DELTA or
                heading_delta > self._PUBLISH_DELTA)

if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("pf_localisation")
    node = ParticleFilterLocalisationNode()
    rospy.spin()