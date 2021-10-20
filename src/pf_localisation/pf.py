from geometry_msgs.msg import Pose, PoseArray, Quaternion
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
from random import random, gauss

from time import time


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
    def init_random_pose(self, mean_pose, sig):
        pose = Pose()
        pose.position = self.init_random_position(mean_pose.position, sig)
        pose.orientation = self.init_random_orientation(mean_pose.orientation, sig)
        return pose

    def init_random_position(self, mean_pos, sig):
        from geometry_msgs.msg import Point
        p = Point()
        p.x = gauss(mean_pos.x, sig)
        p.y = gauss(mean_pos.y, sig)
        p.z = gauss(mean_pos.z, sig)
        return p
    def init_random_orientation(self,mean_ori, sig):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.w = gauss(mean_ori.w, sig)
        return q
        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
    
        N = 12 #number of particles
        sig = 0.1 # sigma of noise gaussian
        pose_array = PoseArray()
        for i in range(N):
            pose_array.posesappend(self.init_random_pose(initialpose.pose.pose, 0.1))
        print(pose_array)
        return pose_array

    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        
        pass

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        return Pose()
