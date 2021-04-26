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

from copy import deepcopy

from likelihood_field import LikelihoodField

# Import functions from other files
def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob

# Given an index and info about map, compute its real coordinate 
def convert_to_real_coords(indx, height, orx, ory, res):
    
    # Convert the x and y indexes from row-major order
    x_val = indx % height 
    y_val = math.floor(indx/height)

    # Scale our x and y indexes to the size and resolution of our map
    x_coord = orx + (x_val * res)
    y_coord = ory + (y_val * res)

    # Create a new coordinate
    coords = [x_coord, y_coord]

    return(coords)


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

def draw_random_sample(cloud,probs, n):
    """ Return a random sample of n elements from the set choices with the specified probabilities
        choices: the values to sample from represented as a list
        probabilities: the probability of selecting each element in choices represented as a list
        n: the number of samples
    """
    # Create a new empty cloud 
    new_cloud = []

    # Create a cloud that stores the integers 1-n
    fake_cloud = np.array(range(n))

    # Create bins from the cumulative sums of our probabilities
    bins = np.cumsum(probs)

    # Create a new array using the np.searchsorted function
    new_array = fake_cloud[np.searchsorted(bins, random_sample(n), side='left')]

    # For each index in our new array, add that particle from our original particle cloud to our new array
    for part in new_array:
        new_cloud.append(deepcopy(cloud[part]))

    return new_cloud

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
        self.likelihood_field = None 
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

        rospy.sleep(1)

        #self.likelihood_field = LikelihoodField()

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)
    


        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        rospy.sleep(2)
        self.initialize_particle_cloud()

        self.initialized = True


    def get_map(self, data):
        self.likelihood_field = LikelihoodField(data)

        self.map = data
    

    def initialize_particle_cloud(self):
        # Figure out height and width of map 
        width = self.map.info.width
        height = self.map.info.height

        # Map is a 384 x 384 grid
        # Create a new array that stores the locations of all points on map that are empty
        full_array = []
        for i in range(width):
            for j in range(height):
                indx = (i*width+j)
                if self.map.data[indx] == 0:
                    full_array.append(indx)

        # Set all our initial particles to empty
        initial_particle_set = []

        # Create a random sample of size n from all the empty locations on map
        new_sample = rand.sample(full_array, self.num_particles)

        # Tranform each point into real coordinates and add a randomized orientation
        for part in new_sample:
            # Find the origin and resolution of the map
            # Recalculate the x and y values of the randomly sampled points to fit our map
            resolution = self.map.info.resolution
            x_origin = self.map.info.origin.position.x
            y_origin = self.map.info.origin.position.y
            part = convert_to_real_coords(part, width, x_origin, y_origin, resolution)

            # Randomly choose an orientation for each particle between 0 and 2 pi
            rand_orientation = math.radians(randint(0,359))
            part.append(rand_orientation)

            # Add particle to our initial particle set
            initial_particle_set.append(part)

        # Initialize our particle cloud to be the size of our map
        self.particle_cloud = []

        # Use code from class to convert to particle type
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

            # Initialize the new particle, where all will have the same weight (1.0)
            new_particle = Particle(p, 1.0)

            # Append the particle to the particle cloud
            self.particle_cloud.append(new_particle)

        # END
        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        # Make all the particle weights sum to 1.0

        # Create sum object to hold total weights
        sum = 0
        for part in self.particle_cloud:
            sum += part.w

        # Make sure we dont divide by zero
        if sum != 0:
            # Re-weigh each particle (normalize them)
            for part in self.particle_cloud:
                part.w = part.w / sum

    def publish_particle_cloud(self):
        rospy.sleep(1)
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
        # Create 1D array of weights to feed to our draw_random_sample function
        weights = []
        for part in self.particle_cloud:
            weights.append(part.w)
        
        # Use draw_random_sample to create a new particle cloud
        self.particle_cloud = draw_random_sample(self.particle_cloud, np.array(weights), self.num_particles) 

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


        if self.particle_cloud:

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
        # Based on the particles within the particle cloud, update the robot pose estimate
        # Use average location of all particles

        # Find total x and y locations
        totalx = 0
        totaly = 0
        total_yaw = 0
        for part in self.particle_cloud:
            # Accumulate position
            pos = part.pose.position
            totalx += pos.x
            totaly += pos.y

            # Accumulate orientation
            p = part.pose
            total_yaw += get_yaw_from_pose(p)

        # Calculate new locations
        new_x = totalx / self.num_particles
        new_y = totaly / self.num_particles

        # Calculate new orientation
        new_yaw = total_yaw / self.num_particles
        new_quat = quaternion_from_euler(0.0, 0.0, new_yaw)

        # Set new robot position estimate
        self.robot_estimate.position.x = new_x
        self.robot_estimate.position.y = new_y

        # Set new robot orientation estimate
        self.robot_estimate.orientation.z = new_quat[2]
        self.robot_estimate.orientation.w = new_quat[3]

    
    def update_particle_weights_with_measurement_model(self, data):
        # Check that likelihood_field is loaded from map
        if self.likelihood_field == None:
            return 
        else: 
            # Should use this for full list of directions
            allDirections = range(360)

            # Set cardinal directions
            cardinal_directions_idxs = [0, 90, 180, 270]

            # Code taken from in-class exercise
            for part in self.particle_cloud:
                q = 1 
                for direction in cardinal_directions_idxs:
                    theta = get_yaw_from_pose(part.pose)
                    ztk = data.ranges[direction]
                    if(data.ranges[direction] <= 3.5):
                        xztk = part.pose.position.x + (ztk * math.cos(theta + math.radians(direction)))
                        yztk = part.pose.position.y  + (ztk * math.sin(theta + math.radians(direction)))
                        dist = self.likelihood_field.get_closest_obstacle_distance(xztk, yztk)
                        prob = compute_prob_zero_centered_gaussian(dist, 0.1)
                        # Only modify q if the probability is not a nan value
                        if not (math.isnan(prob)):
                            q = q * prob
                # If q is a nan value or 1 (i.e. hasnt been changed at all), set q to 0
                if (math.isnan(q)) or q == 1:
                    q = 0
                # Update particle weight
                part.w = q 
        

    def update_particles_with_motion_model(self):
        # Find old and new robot position and orientation
        # Calculate distance moved in x direction
        x_old = self.odom_pose_last_motion_update.pose.position.x
        x_new = self.odom_pose.pose.position.x
        delta_x = x_new - x_old

        # Calculate distance moved in y direction
        y_old = self.odom_pose_last_motion_update.pose.position.y
        y_new =  self.odom_pose.pose.position.y
        delta_y = y_new - y_old 

        # Calculate rotation 
        yaw_old = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
        yaw_new = get_yaw_from_pose(self.odom_pose.pose)
        delta_yaw = yaw_new - yaw_old 

        # Calculate total distance moved using pythagorean thm
        total_dist = math.sqrt(delta_x*delta_x+delta_y*delta_y)
       
        # Based on the how the robot has moved (calculated from its odometry), 
        # move all of the particles correspondingly

        # Modify each particle direction and orientation based on how robot has moved
        # Move each particle the same distance and direction as robot respective to its orientation
        for part in self.particle_cloud:

            # Find particle orientation and add noise
            theta = get_yaw_from_pose(part.pose)
            theta_with_noise = np.random.normal(loc = (theta), scale = 0.2)
            # Add the robots direction change and convert back to a quaternion
            quat_noise = quaternion_from_euler(0, 0, theta_with_noise + delta_yaw)

            # Calculate the angle between the robot and particles orientation and 
            # Calculate the respective x and y directions using that angle
            theta_diff = theta - yaw_old
            new_x = (delta_x * math.cos(theta_diff)) + (delta_y * math.sin(theta_diff))
            new_y = (-delta_x * math.sin(theta_diff)) + (delta_y * math.cos(theta_diff))
            
            # Update the particles location and orientation
            part.pose.position.x += np.random.normal(loc = new_x, scale=0.2)
            part.pose.position.y += np.random.normal(loc = new_y, scale=0.2) 
            part.pose.orientation.x = quat_noise[0]
            part.pose.orientation.y = quat_noise[1]
            part.pose.orientation.z = quat_noise[2]
            part.pose.orientation.w = quat_noise[3]


if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









