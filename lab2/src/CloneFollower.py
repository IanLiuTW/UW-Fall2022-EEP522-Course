#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

import Utils  # <----------- LOOK AT THESE FUNCTIONS ***************************

SUB_TOPIC = '/car/car_pose'  # The topic that provides the simulated car pose
PUB_TOPIC = '/clone_follower_pose/pose'  # The topic that you should publish to
MAP_TOPIC = '/static_map'  # The service topic that will provide the map

# Follows the simulated robot around


class CloneFollower:

    '''
    Initializes a CloneFollower object
    In:
      follow_offset: The required x offset between the robot and its clone follower
      force_in_bounds: Whether the clone should toggle between following in front
                        and behind when it goes out of bounds of the map
    '''

    def __init__(self, follow_offset, force_in_bounds):
        # YOUR CODE HERE
        self.follow_offset = follow_offset  # Store the input params in self
        self.force_in_bounds = force_in_bounds  # Store the input params in self
        self.map_img, self.map_info = Utils.get_map(
            MAP_TOPIC)  # Get and store the map
        # for bounds checking

        # Setup publisher that publishes to PUB_TOPIC
        self.pub = rospy.Publisher(PUB_TOPIC, PoseStamped, queue_size=10)

        # Setup subscriber that subscribes to SUB_TOPIC and uses the self.update_pose
        # callback
        self.sub = rospy.Subscriber(SUB_TOPIC, PoseStamped, self.update_pose)

    '''
    Given the translation and rotation between the robot and map, computes the pose
    of the clone
    (This function is optional)
    In:
        trans: The translation between the robot and map
        rot: The rotation between the robot and map
    Out:
        The pose of the clone
    '''

    def compute_follow_pose(self, trans, rot):
        # YOUR CODE HERE
        pass

    '''
    Callback that runs each time a sim pose is received. Should publish an updated
    pose of the clone.
    In:
        msg: The pose of the simulated car. Should be a geometry_msgs/PoseStamped
    '''

    def update_pose(self, msg):
        # YOUR CODE HERE
        # Compute the pose of the clone
        # Note: To convert from a message quaternion to corresponding rotation matrix,
        #       look at the functions in Utils.py
        position, orientation = msg.pose.position, msg.pose.orientation
        yaw = Utils.quaternion_to_angle(orientation)
        rot_mat = Utils.rotation_matrix(yaw)
        new_x = position.x + self.follow_offset*rot_mat[0, 0]
        new_y = position.y + self.follow_offset*rot_mat[1, 0]
        # print(position.x, position.y)
        # print(new_x, new_y)

        # Check bounds if required
        if self.force_in_bounds:
            # Functions in Utils.py will again be useful here
            config = Utils.world_to_map(
                [new_x, new_y, position.z], self.map_info)
            # print(config)
            # print(self.map_img[config[0], config[1]])
            if self.map_img[config[0], config[1]] == 0:
                self.follow_offset = -self.follow_offset
                rospy.set_param("follow_offset", self.follow_offset)

        # Setup the out going PoseStamped message
        new_pose = PoseStamped()
        new_pose.header.stamp, new_pose.header.frame_id = rospy.Time.now(), msg.header.frame_id
        new_pose.pose.position.x, new_pose.pose.position.y = new_x, new_y
        new_pose.pose.orientation = orientation

        # Publish the clone's pose
        self.pub.publish(new_pose)


if __name__ == '__main__':
    follow_offset = 1.0  # The offset between the robot and clone
    force_in_bounds = False  # Whether or not map bounds should be enforced

    rospy.init_node('clone_follower', anonymous=True)  # Initialize the node

    # Populate params with values passed by launch file
    follow_offset = rospy.get_param("follow_offset")  # YOUR CODE HERE
    force_in_bounds = rospy.get_param("force_in_bounds")  # YOUR CODE HERE

    # Create a clone follower
    cf = CloneFollower(follow_offset, force_in_bounds)
    rospy.spin()  # Spin
