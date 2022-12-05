#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

PUB_TOPIC = '/car/mux/ackermann_cmd_mux/input/teleop'
PUB_RATE = 5


def follow_bag():
    pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=100)
    rate = rospy.Rate(PUB_RATE)

    ads = AckermannDriveStamped()
    ads.drive.speed = -2.0
    for _ in range(5):
        pub.publish(ads)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('reverse', anonymous=True)
    follow_bag()
