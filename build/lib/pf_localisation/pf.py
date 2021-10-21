from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
from random import random, gauss
from time import time,sleep


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        self.number_of_particles = 200
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
        p = Point()
        p.x = gauss(mean_pos.x, sig)
        p.y = gauss(mean_pos.y, sig)
        return p
    def init_random_orientation(self,mean_ori, sig):
        return rotateQuaternion(mean_ori, (2* math.pi * random()) -math.pi )
        
        
    
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
    
        N = self.number_of_particles #number of particles
        sig =8  # sigma of noise gaussian
        pose_array = PoseArray()
        for i in range(N):
            pose_array.poses.append(self.init_random_pose(initialpose.pose.pose, sig))
        
       
        pose_array.header = initialpose.header
        
        
        return pose_array

    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """

        desired_particles_num = self.number_of_particles 
        weight_data = []
        # ----- Compute the likelihood weighting for each of a set of particles
        

        self.particlecloud.poses.sort(key= lambda p: self.sensor_model.get_weight(scan, p) )
        for p in self.particlecloud.poses:
            weight_data.append(self.sensor_model.get_weight(scan, p))
        
        weight_data = self.normalise(weight_data)
        cdf = [weight_data[0]]
        
        for i in range(1,len(weight_data)):
            cdf.append(cdf[i-1] + weight_data[i])
        
        threshold = math.pow(desired_particles_num,-1)
        i=1
        new_particle_cloud = PoseArray()
        for j in range(0, desired_particles_num):
            while(threshold > cdf[i]):
                i+=1
                if i == len(cdf):
                    self.particlecloud.poses = new_particle_cloud.poses
                    pub = rospy.Publisher('/particlecloud', PoseArray, queue_size=50)
                    pub.publish(self.particlecloud)
                    print(self.particlecloud.poses[0])
                    return
            new_particle_cloud.poses.append(self.particlecloud.poses[i])
            
            '''if j+1 == len(threshold):
                threshold.append(threshold[j] + math.pow(desired_particles_num,-1))
            else:
                threshold[j+1] = threshold[j] + math.pow(desired_particles_num,-1)
            '''
            threshold = threshold + math.pow(desired_particles_num,-1)
<<<<<<< Updated upstream
        self.particlecloud.poses = new_particle_cloud.poses
        pub = rospy.Publisher('/particlecloud', PoseArray, queue_size=50)
        pub.publish(self.particlecloud)
        print(self.particlecloud.poses[0])
=======
            print('Threshold', threshold,'cdf i', cdf[i])
        print(cloned_particles)
        self.particlecloud = new_particle_cloud
>>>>>>> Stashed changes
        
        
    def normalise(self, l):
        total = 0
        for i in range(len(l)):
            total +=l[i]
        print(total)
        for a in range(len(l)):
            l[a] = l[a]/ total
        return l
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
