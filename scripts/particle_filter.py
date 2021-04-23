#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math
import random as rand 

from random import randint, random

# Import functions from other files
from likelihood_field import LikelihoodField
def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob

#given an index and info about map, compute its real coordinate 
def convert_to_real_coords(indx, height, orx, ory, res):
    
    x_val = indx % height 
    y_val = math.floor(indx/height)

    x_coord = orx + (x_val * res)
    y_coord = ory + (y_val * res)
    coords = [x_coord, y_coord]
    return(coords)

#test function, should return -9.7, -9.95
#print(convert_to_real_coords(390, 384, -10, -10, 0.05))

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

#here, lst = lst of particle positions, n = 10,0000, probs = list of particle weights 
def draw_random_sample(cloud,probs,n):
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
    # All test code -------
    sum = 0
    for part in cloud:
        sum += part.w
    print("original sum = " + str(sum))
    # ---------------------

    # create a new array of n sampled particles from our cloud
    new_cloud = rand.choices(cloud, weights = probs, k=n) 

    # All test code -------
    sum = 0
    for part in new_cloud:
        sum += part.w
        #print(part.w)
    print("new sum = " + str(sum))
    # ---------------------

    return new_cloud

#let's test this boy 
# test_cloud = [5, 19, 23, 67, 100, 245, 316, 325, 388, 567, 891, 905, 936, 999]
# test_probs = [0, 1, 8, 0, 5, 10, 0, 0, 1, 0, 2, 0, 15, 0]
# print(draw_random_sample(test_cloud, test_probs, 14))

class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w

class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # haha are we supposed to do this part 3
        self.likelihood_field = LikelihoodField()

        # the number of particles used in the particle filter
        self.num_particles = 10000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True


    def get_map(self, data):

        self.map = data
    

    def initialize_particle_cloud(self):

        # TODO 
        # Figure out height and width of world 
        width = self.map.info.width
        height = self.map.info.height
        print(height*width)
        total = 0
        for i in range(width * height):
            if self.map.data[i] == 0:
                total += 1
        print(total)

        #map is a 384 x 384 grid

        # create a scale to normalize our particles by 
        w_scale = width // 100
        h_scale = height // 100

        # create a new array that stores the locations of points that are empty
        full_array = []
        for i in range(width):
            for j in range(height):
                #print(i*width+j)
                indx = (i*width+j)
                if self.map.data[indx] == 0:
                    #changed this to append index rather than value 
                    full_array.append(indx)

        initial_particle_set = []

        # set all our initial particles
        new_sample = rand.sample(full_array, 10000)

        for part in new_sample:
            #indx, height, orx, ory, res just hard coding in args for now 
            part = convert_to_real_coords(part, 384, -10, -10, 0.05)
            rand_orientation = np.random.uniform(0, (2 * math.pi))
            part.append(rand_orientation)
            initial_particle_set.append(part)

        #print(initial_particle_set)
        # Initialize our particle cloud to be the size of our map
        self.particle_cloud = []

        for i in range(len(initial_particle_set)):
            p = Pose()
            p.position = Point()
            p.position.x = initial_particle_set[i][0]
            p.position.y = initial_particle_set[i][1]
            p.position.z = 0
            p.orientation = Quaternion()
            q = quaternion_from_euler(0.0, 0.0, initial_particle_set[i][2])
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            # initialize the new particle, where all will have the same weight (1.0)
            new_particle = Particle(p, 1.0)

            # append the particle to the particle cloud
            self.particle_cloud.append(new_particle)

        # END
        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        
        # TODO

        # create sum object to hold total weights
        sum = 0
        for part in self.particle_cloud:
            sum += part.w

        # make sure we dont divide by zero
        if sum != 0:
            # Re-weigh each particle (normalize them)
            for part in self.particle_cloud:
                part.w = part.w / sum
                #print(part.w) # this is useless

        print("sum = " + str(sum)) # test normalize particles fn

    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses
        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)



    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):

        # TODO

        # create 1D array of weights
        weights = []
        check_sum = 0 # tester variable
        for part in self.particle_cloud:
            weights.append(part.w)
            check_sum += part.w
        
        #print("total weight: " + str(check_sum)) # test if weights sum to 1 or not

        # use draw_random_sample to create a new particle cloud
        new_cloud = draw_random_sample(self.particle_cloud, weights, 10000)
        self.particle_cloud = new_cloud

    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        #use this important
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        #this is how far robot has moved
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud != []:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        # use average location of all particles
        # TODO

        #find total x and y locations
        totalx = 0
        totaly = 0
        totalO = [0,0,0,0] # use eulers here
        for part in self.particle_cloud:
            pos = part.pose.position
            totalx += pos.x
            totaly += pos.y

            # this is also wrong use eulers again
            p = part.pose
            totalO[0] += p.orientation.x
            totalO[1] += p.orientation.y
            totalO[2] += p.orientation.z
            totalO[3] += p.orientation.w 

        # calculate new locations
        new_o = [totalO[0] / 10000, totalO[1] / 10000, totalO[2] / 10000, totalO[3] / 10000] # wrong
        new_x = totalx / 10000
        new_y = totaly / 10000
        #print("x = " + str(new_x) + "\ny = " + str(new_y)+ "\norientation = " + str(new_o))

        # set new robot estimate
        self.robot_estimate.position.x = new_x
        self.robot_estimate.position.y = new_y

        # use eulers
        self.robot_estimate.orientation.x = new_o[0]
        self.robot_estimate.orientation.y = new_o[1]
        self.robot_estimate.orientation.z = new_o[2]
        self.robot_estimate.orientation.w = new_o[3]


    
    def update_particle_weights_with_measurement_model(self, data):

        # TODO

        # Should use this for full list of directions
        allDirections = range(360)

        # Starting with cardinal directions to save time
        cardinal_directions_idxs = [0, 45, 90, 120, 180, 225, 270, 315]

        # Code taken from in-class exercise
        for particle in self.particle_cloud:
            q = 1 
            for direction in cardinal_directions_idxs:
                quat_array = []
                quat_array.append(particle.pose.orientation.x)
                quat_array.append(particle.pose.orientation.y)
                quat_array.append(particle.pose.orientation.z)
                quat_array.append(particle.pose.orientation.w)
                euler_points = euler_from_quaternion(quat_array)
                theta = euler_points[2] 
                ztk = data.ranges[direction]
                if(data.ranges[direction] <= 3.5):
                    xztk = particle.pose.position.x + (ztk * math.cos(theta + math.radians(direction)))
                    yztk = particle.pose.position.y  + (ztk * math.sin(theta + math.radians(direction)))
                    dist = LikelihoodField.get_closest_obstacle_distance(self.likelihood_field, xztk, yztk)
                    q = q * compute_prob_zero_centered_gaussian(dist, 0.1)
                    #particle.w = q 
                    #print(q)
                    # print(direction)
                    # print("dist: " + str(dist))
                    # print("q: " + str(q))
                    # print("\n")
            if (math.isnan(q)):
                #print(q)
                q = 0
            particle.w = q 

        
        # for part in self.particle_cloud:
        #     print(part.w)
                    
            # print(q)
            # if q != "nan":
            #     particle.w = q
            # else:
            #     particle.w = 0
            


    def update_particles_with_motion_model(self):
        # TODO

        x_old = self.odom_pose_last_motion_update.pose.position.x
        x_new = self.odom_pose.pose.position.x
        delta_x = x_new - x_old 

        y_old = self.odom_pose_last_motion_update.pose.position.y
        y_new =  self.odom_pose.pose.position.y
        delta_y = y_new - y_old 

        yaw_old = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
        yaw_new = get_yaw_from_pose(self.odom_pose.pose)
        delta_yaw = yaw_new - yaw_old 

        yaw_to_quant = quaternion_from_euler(0.0, 0.0, delta_yaw)

        # We need to use delta yaws here
        #do some heccin trig to figure out how much to move 
        
        
       
        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        #not sure if I am iterating thru this right 
        for part in self.particle_cloud:

            theta = part.pose.position.z 
            #rotation of axis, assume ccw ig 
            new_x = (delta_x * math.cos(theta)) + (delta_y * math.sin(theta))
            new_y = (-delta_x * math.sin(theta)) + (delta_y * math.cos(theta))

            part.pose.position.x += new_x 
            part.pose.position.y += new_y 
            part.pose.position.z += yaw_to_quant

            # this is wrong: convert to yaws
            #print(part.pose.orientation)
            #print(yaw_to_quant)
            #part.pose.orientation.z += yaw_to_quant[2]


        # ADD NOISE HERE

        # Add position noise
        
        # Add orientation noise
'''
    
def testy_boi(lst, delta_x, delta_y):
    for el in lst: 
        theta = el[2]
        #rotation of axis, assume ccw ig 
        new_x = (delta_x * math.cos(theta)) + (delta_y * math.sin(theta))
        new_y = (-delta_x * math.sin(theta)) + (delta_y * math.cos(theta))

        el[0] += new_x 
        el[1] += new_y 
        #el[2] += yaw_to_quant
    return lst 

test_lst = [[0,0,1.5], [2,3,0.5], [-5,-6, 6.1], [8,9,4.3]]
'''
'''
expect 
4.2, -2.7
6.5, 5.07 
-2.7, -1.5 
3.2, 10.1
'''

#print(testy_boi(test_lst, 3, 4))



if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









