#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import copy
import numpy as np
import open3d as o3
from probreg import cpd
from probreg import gmmtree
import time
def data_transform(data):
    global pc_list, last_time, current_time
    # last_time=current_time

    # print('aaaa')
    pc = pc2.read_points(data, skip_nans=True)
    pc_list = np.arange(6).reshape(2,3)
    for p in pc:
        pc_list=np.append(pc_list, [[p[0],p[1],p[2]]], axis=0 )
    # print(pc_list)
    pc_list=np.delete(pc_list,0,0)
    pc_list=np.delete(pc_list,0,0)
    # print('bbbbbbb')
    # print(type(pc_list))
    if pc_list.shape[0]==0:
        print("no point received! hgmm construction stops!")
        

    before_mapping=time.time()
    gt = gmmtree.GMMTree(pc_list,tree_level=2)
    after_mapping=time.time()
    print(after_mapping-before_mapping)

    # current_time=time.time()
    # print(current_time-last_time)

def hgmm_build():
    global pc_list, last_time, current_time
    current_time=time.time()
    print('start hgmm building!')
    rospy.init_node('hgmm_building', anonymous=True)
    sub = rospy.Subscriber('/points', PointCloud2, data_transform)
    rospy.spin()


    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()
 
if __name__ == '__main__':
    try:
        hgmm_build()
    except rospy.ROSInterruptException:
        pass