#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped

class nav_goal_transform_node:

    def __init__(self):
        rospy.init_node('uav_tf_node', anonymous=True)
        self.x_pos_init_ = rospy.get_param('/x_pos_init')
        self.y_pos_init_ = rospy.get_param('/y_pos_init')
        #self.z_pos_init_ = rospy.get_param('/z_pos_init')
        self.targetSub_ = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.targetCallback,queue_size=100)
        self.targetmodifiedPub_ = rospy.Publisher('/move_base_simple/goal_modified', PoseStamped, queue_size=100)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass

    def targetCallback(self, msg):
        msg.pose.position.x -= self.x_pos_init_
        msg.pose.position.y -= self.y_pos_init_
        self.targetmodifiedPub_.publish(msg)


if __name__ == '__main__':
    ngtn = nav_goal_transform_node()
