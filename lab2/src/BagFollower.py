#!/usr/bin/env python

import rosbag
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

# Name of the topic that should be extracted from the bag
BAG_TOPIC = '/car/mux/ackermann_cmd_mux/input/teleop'
PUB_TOPIC = '/car/mux/ackermann_cmd_mux/input/teleop'
PUB_RATE = 5  # The rate at which messages should be published

# Loads a bag file, reads the msgs from the specified topic, and republishes them


def follow_bag(bag_path, follow_backwards=False):
    pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=100)
    rate = rospy.Rate(PUB_RATE)
    bag = rosbag.Bag(bag_path)

    msgs = []
    for topic, msg, t in bag.read_messages(topics=[BAG_TOPIC]):
        msgs.append(msg)
        # print(topic, t)
    bag.close()

    if follow_backwards:
        msgs.reverse()
        for msg in msgs:
            msg.drive.jerk *= -1
            msg.drive.acceleration *= -1
            msg.drive.speed *= -1

    for msg in msgs:
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    bag_path = None  # The file path to the bag file
    follow_backwards = False  # Whether or not the path should be followed backwards

    rospy.init_node('bag_follower', anonymous=True)

    # Populate param(s) with value(s) passed by launch file
    bag_path = rospy.get_param("bag_path")
    follow_backwards = rospy.get_param("follow_backwards")

    follow_bag(bag_path, follow_backwards)

