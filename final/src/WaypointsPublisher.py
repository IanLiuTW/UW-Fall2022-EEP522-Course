#!/usr/bin/env python

import rospy
import numpy as np
from final.srv import *
import Putils
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped
from final_waypoints import WAYPOINTS
from collections import deque

TARGET_TOPIC = '/move_base_simple/goal'

def generate_pose_msg(pose):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = pose[0]
    msg.pose.position.y = pose[1]
    return msg

def main():
    rospy.init_node('waypoints_publisher', anonymous=True)
    map_service_name = rospy.get_param("~static_map", "static_map")
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    map_info = rospy.ServiceProxy(map_service_name, GetMap)().map.info

    target_pub = rospy.Publisher(TARGET_TOPIC, PoseStamped, queue_size=10)
    # for p in WAYPOINTS:
    #     print(Putils.map_to_world(p, map_info))
    # rate = rospy.Rate(1.5)

    queue = deque(WAYPOINTS)
    while queue and not rospy.is_shutdown():
        print('Queue length:', len(queue))
        raw_input("Wait for inserting...")
        p = queue.popleft()
        pose_msg = generate_pose_msg(Putils.map_to_world(p, map_info))
        print(pose_msg)
        target_pub.publish(pose_msg)
        # rate.sleep()
        
        
if __name__ == '__main__':
    main()
