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
        q = Quaternion()
        q.z = gauss(mean_ori.z,sig)
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
    
        N = self.number_of_particles #number of particles
        sig =1  # sigma of noise gaussian
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
        

        #self.particlecloud.poses.sort(key= lambda p: self.sensor_model.get_weight(scan, p) )
        for p in self.particlecloud.poses:
            weight_data.append(self.sensor_model.get_weight(scan, p))
            #print(self.sensor_model.get_weight(scan, p))
        
        weight_data = self.normalise(weight_data)
        cdf = [weight_data[0]]
        
        for i in range(1,len(weight_data)):
            cdf.append(cdf[i-1] + weight_data[i])
        
        threshold = random()* math.pow(desired_particles_num,-1)
        i=1
        new_particle_cloud = PoseArray()
        #print('CDF',cdf)
        
        for j in range(0, desired_particles_num):
            while(threshold > cdf[i]):
                i+=1
            new_particle_cloud.poses.append(self.init_random_pose(self.particlecloud.poses[i],0.3))
            
            '''if j+1 == len(threshold):
                threshold.append(threshold[j] + math.pow(desired_particles_num,-1))
            else:
                threshold[j+1] = threshold[j] + math.pow(desired_particles_num,-1)
            '''
            threshold = threshold + math.pow(desired_particles_num,-1)
         #   print('Threshold', threshold,'cdf i', cdf[i])
        #print(cloned_particles)
        self.particlecloud = new_particle_cloud

        #print(self.particlecloud.poses[0])
        #input()
        
    def normalise(self, l):
        total = 0
        for i in range(len(l)):
            total +=l[i]
        
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

        # Implementing hierarchical clustering

        def calculate_distance(mean1, mean2):
            return math.sqrt(math.pow(mean1[0] - mean2[0],2) + math.pow(mean1[1] - mean2[1],2))

        def calculate_mean(cluster):
            mean_x = 0
            mean_y = 0
            for i in cluster:
                mean_x += i.x
                mean_y += i.y
            len_cluster = len(cluster)
            mean_x /= len_cluster
            mean_y /= len_cluster
            return mean_x, mean_y, len_cluster

        # Each pose is a cluster itself
        cluster_list = [] # list of clusters, a cluster is a list of poses
        cluster_data_list = [] # list of the means of the clusters
        number_clusters = 0
        for p in self.particlecloud.poses:
            cluster_list.append([p])
            cluster_data_list.append((p.position.x, p.position.y, 1))
            number_clusters += 1

        while number_clusters > 2:
            lowest_distance = float('inf')
            cluster1 = -42
            cluster2 = -427
            # Finding the two closest clusters
            for i in range(number_clusters):
                for j in range(number_clusters):
                    if i > j:
                        distance = calculate_distance(cluster_data_list[i], cluster_data_list[j])
                        if distance < lowest_distance:
                            lowest_distance = distance
                            cluster1 = i
                            cluster2 = j
            if lowest_distance < 0.5:
                break
            # Merging the two closest clusters
            number_clusters -= 2
            cluster_list[cluster1].append(cluster_list[cluster2])
            cluster_list.pop(cluster2)
            cluster_data_list.pop(cluster2)
            # Calculate new mean of the cluster
            cluster_data_list[cluster1] = calculate_mean(cluster_list[cluster1])

        # Computes the tallest cluster
        tallest_cluster_index = -42
        for c in cluster_data_list:
            if c[2] > tallest_cluster_index:
                tallest_cluster_index = c[2]

        pose = Pose()
        p = Point()
        p.x = cluster_data_list[tallest_cluster_index][0]
        p.y = cluster_data_list[tallest_cluster_index][1]

        tallest_cluster = cluster_list[tallest_cluster_index]
        sum_zr = sum_wr = 0
        for c in  tallest_cluster:
            sum_zr+= c.orientation.z
            sum_wr+= c.orientation.w
        q = Quaternion()
        len_tallest_cluster = len(tallest_cluster)
        q.z = sum_zr/len_tallest_cluster
        q.w = sum_wr/len_tallest_cluster
        pose.position = p
        pose.orientation = q
        # Todo: Compute the mean of the quaternions
        
        return pose
