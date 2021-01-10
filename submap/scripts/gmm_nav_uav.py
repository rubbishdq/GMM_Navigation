#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from collections import deque
import signal
import sys
import rospy
import copy
import numpy as np
import scipy.stats as st
import math
import threading

from geometry_msgs.msg import Point,Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Path
from submap.msg import gmm, gmmlist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

# gmm_cnt=0
# goal_cnt=0

def subgmm_cb(data):
    global gmm_cnt
    global gmm_map
    gmm_cnt=gmm_cnt+1
    gmm_map=data
    # if (gmm_cnt!=0):
    #     if(data.prior[1]!=prev_data.prior[1]):
    #         gmm_cnt=gmm_cnt+1
    #         gmm_map=data
    # else:
    #     gmm_map=data
    #     gmm_cnt=gmm_cnt+1

    # target is the ultimate navigation point, goal is the next navigation step    
def goal_cb(data):
    global goal_cnt, target, target_deque
    if goal_cnt == 0:
        target=[data.pose.position.x,data.pose.position.y]
    else:
        target_deque.append([data.pose.position.x,data.pose.position.y])
    goal_cnt=goal_cnt+1

'''
def begin_cb(data):
    global begin_pos, begin_quat
    if 'iris' in data.name:
        i = data.name.index('iris')
        pose = data.pose[i]
        begin_pos=[pose.position.x,pose.position.y]
        # print(begin_pos[0], begin_pos[1])
        begin_quat=[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
'''

def mavpose_cb(data):
    global begin_pos, begin_quat
    begin_pos=[data.pose.position.x, data.pose.position.y, data.pose.position.z]
    begin_quat=[data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]

def mavros_state_cb(data):
    global fcu_state
    fcu_state=data

def prob_visual(gmm_map):
    global resulotion, z_const
    Xbottom=gmm_map.x[gmm_map.x.index(min(gmm_map.x))]-2  # is gmm_map.x a list?
    Xtop=gmm_map.x[gmm_map.x.index(max(gmm_map.x))]+2
    Ybottom=gmm_map.y[gmm_map.y.index(min(gmm_map.y))]-2
    Ytop=gmm_map.y[gmm_map.y.index(max(gmm_map.y))]+2
    pro=np.zeros((int((Xtop-Xbottom)/resolution+1),int((Ytop-Ybottom)/resolution+1)))
    num_tmp=0
    for i in range(0,gmm_map.mix_num):
        if (gmm_map.z[i]>0.1):
            num_tmp=num_tmp+1
    mu=np.zeros((num_tmp,3))
    sigma=np.zeros((num_tmp,3,3))
    prior=np.zeros((num_tmp,1))
    ptr=0
    for p in range(0,gmm_map.mix_num):
        if (gmm_map.z[p]>0.1):
            mu[ptr,:]=[gmm_map.x[p],gmm_map.y[p],gmm_map.z[p]]
            sigma[ptr,:,:]=np.diag([gmm_map.x_var[p],gmm_map.y_var[p],gmm_map.z_var[p]])
            prior[ptr]=gmm_map.prior[p]
            ptr=ptr+1
    ptr=0
    # print("mu:",mu)
    # print("sigma:",sigma)
    # print("prior:",prior)
    for x in np.arange(Xbottom,Xtop,resolution):
        y= np.arange(Ybottom,Ytop,resolution)
        y=y.reshape(len(y),1)
        # print(len(y))
        # print("---------------")        
        L=int((Ytop-Ybottom)/resolution+1)
        # print(L)
        x_temp=np.ones([L,1])*x
        z_temp=np.ones([L,1])*z_const
        pt_temp=np.hstack((x_temp,y,z_temp))
        totalpdf=np.zeros((L,1))
        for pp in range(0,num_tmp):
            Npdf=st.multivariate_normal.pdf(pt_temp,mu[pp,:],sigma[pp,:,:])
            # print(Npdf)
            # print("-------------------------------")
            # print(np.size(Npdf))
            totalpdf=totalpdf+Npdf.reshape(len(Npdf),1)*prior[pp]
        # print(totalpdf)
        # print(pro[ptr,:].shape)
        pro[ptr,:]=totalpdf.reshape(pro[ptr,:].shape)
        # print("ptr: ", ptr)
        # print("pro[ptr,:]: ", totalpdf.reshape(pro[ptr,:].shape))
        ptr=ptr+1
    # print("!!!!!!!!!!!!!!!!!!!!!!!!")
    # print(pro)
    # print("!!!!!!!!!!!!!!!!!!!!!!!!")
    return pro,Xtop,Xbottom,Ytop,Ybottom

def gmm_nav():
    global z_const, gmm_map, resolution, begin_pos, begin_quat, target, move_pub, path_pub, tra_total, ang_total
    global uav_local_target, height, target_lock
    global goal_cnt, target_deque
    [pro,Xtop,Xbottom,Ytop,Ybottom]=prob_visual(gmm_map)
    # begin=[int((begin_pos[0]-Xbottom)/resolution),int((begin_pos[1]-Ybottom)/resolution)]
    #print('Xbottom: ', Xbottom)
    #print('Ybottom: ', Ybottom)
    #print('Xtop: ', Xtop)
    #print('Ytop: ', Ytop)
    radius=0.2
    [Fx,Fy]=np.gradient(pro)
    # print("Fx: ", Fx)
    # print("Fy: ", Fy)
    # F1_tmp=Fx[int((begin_pos[0]-Xbottom)/resolution),int((begin_pos[1]-Ybottom)/resolution)]
    # F2_tmp=Fy[int((begin_pos[0]-Xbottom)/resolution),int((begin_pos[1]-Ybottom)/resolution)]
    # F_prev=np.sqrt(pow(F1_tmp,2)+pow(F2_tmp,2))
    angle_ulti=math.atan2((target[1]-begin_pos[1]),(target[0]-begin_pos[0]))
    # while(true):
    dist = np.sqrt(pow(target[0]-begin_pos[0],2)+pow(target[1]-begin_pos[1],2))
    goal=[begin_pos[0]+min(dist, radius)*math.cos(angle_ulti),begin_pos[1]+min(dist, radius)*math.sin(angle_ulti)] # position in the real world
    F1_tmp=Fx[int((goal[0]-Xbottom)/resolution),int((goal[1]-Ybottom)/resolution)]
    F2_tmp=Fy[int((goal[0]-Xbottom)/resolution),int((goal[1]-Ybottom)/resolution)]
    angle_x=math.atan2((goal[1]-begin_pos[1]),(goal[0]-begin_pos[0]))/math.pi*180

    F_now=np.sqrt(pow(F1_tmp,2)+pow(F2_tmp,2))
    print("F_now: ", F_now)
    threshold = 1e-25
    flag = False
    if (F_now>threshold):
        for p in range(1,7):
            angle1=angle_x+p*30
            angle1=angle1*math.pi/180
            goal1_tmp=[begin_pos[0]+radius*math.cos(angle1),begin_pos[1]+radius*math.sin(angle1)]
            F1_tmp=Fx[int((goal1_tmp[0]-Xbottom)/resolution),int((goal1_tmp[1]-Ybottom)/resolution)]
            F2_tmp=Fy[int((goal1_tmp[0]-Xbottom)/resolution),int((goal1_tmp[1]-Ybottom)/resolution)]
            F1_now=np.sqrt(pow(F1_tmp,2)+pow(F2_tmp,2))
            print('F1_now from round %d: ' % (p), F1_now)
            angle2=angle_x-p*30
            angle2=angle2*math.pi/180
            goal2_tmp=[begin_pos[0]+radius*math.cos(angle2),begin_pos[1]+radius*math.sin(angle2)]
            F1_tmp=Fx[int((goal2_tmp[0]-Xbottom)/resolution),int((goal2_tmp[1]-Ybottom)/resolution)]
            F2_tmp=Fy[int((goal2_tmp[0]-Xbottom)/resolution),int((goal2_tmp[1]-Ybottom)/resolution)]
            F2_now=np.sqrt(pow(F1_tmp,2)+pow(F2_tmp,2))
            print('F2_now from round %d: ' % (p), F2_now)
            if (min(F1_now,F2_now)<F_now):
                if (F1_now<F2_now):
                    goal=goal1_tmp
                    angle_x=math.atan2((goal[1]-begin_pos[1]),(goal[0]-begin_pos[0]))/math.pi*180
                    # angle_x=math.atan((target[1]-goal[1])/(target[0]-goal[0]))
                    # print("-------------------------------------1", angle1)
                    print('choosing F1_tmp from round %d' % (p))
                else :
                    goal=goal2_tmp
                    angle_x=math.atan2((goal[1]-begin_pos[1]),(goal[0]-begin_pos[0]))/math.pi*180
                    # print("-------------------------------------2", angle2,angle_x)
                    # angle_x=math.atan((target[1]-goal[1])/(target[0]-goal[0]))
                    print('choosing F2_tmp from round %d' % (p))
                flag = True
                break
        print("0000000000000000000")    
    else:
        flag = True
    if not flag: # going backward
        goal = [begin_pos[0]+min(dist, radius)*math.cos(angle_ulti),begin_pos[1]+min(dist, radius)*math.sin(angle_ulti)]
        angle_x = (angle_x+180) if (angle_x<0) else (angle_x-180)
        
    tra_total=tra_total+radius
    [angleX,angleY,angleZ]=quat_to_euler(begin_quat[0],begin_quat[1],begin_quat[2],begin_quat[3])
    ang_total=ang_total+ ang_diff(angle_x, angleZ)       
    print("tra_total= ", tra_total)
    print("ang_total= ", ang_total)

    [angleX,angleY,angleZ]=quat_to_euler(begin_quat[0],begin_quat[1],begin_quat[2],begin_quat[3])


    while (ang_diff(angleZ, angle_x)>2) or (abs(begin_pos[0]-goal[0])>0.09 or abs(begin_pos[1]-goal[1])>0.09):
        #print('diff_yaw, diff_x, diff_y')
        #print((angleZ-angle_x, begin_pos[0]-goal[0], begin_pos[1]-goal[1]))
        target_lock.acquire()
        uav_local_target.pose.position.x = goal[0]
        uav_local_target.pose.position.y = goal[1]
        uav_local_target.pose.position.z = height
        uav_local_target.pose.orientation.x = 0.
        uav_local_target.pose.orientation.y = 0.
        uav_local_target.pose.orientation.z = math.sin(angle_x*math.pi/180/2)
        uav_local_target.pose.orientation.w = math.cos(angle_x*math.pi/180/2)
        target_lock.release()
        rospy.sleep(rospy.Duration(0.1))
        [angleX,angleY,angleZ]=quat_to_euler(begin_quat[0],begin_quat[1],begin_quat[2],begin_quat[3])

    if (abs(begin_pos[0]-target[0])<0.1 and abs(begin_pos[1]-target[1])<0.1):
        goal_cnt -= 1
        if goal_cnt > 0:
            target = target_deque.popleft()

def quat_to_euler(x,y,z,w):
    r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    p = math.asin(2*(w*y-z*x))
    y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))

    angleR = r*180/math.pi #Y
    angleP = p*180/math.pi #X
    angleY = y*180/math.pi #Z
    return angleP,angleR,angleY

# -pi < ang1, ang2 < pi
def ang_diff(ang1, ang2, is_deg=True):
    if ang1 > ang2:
        ang1, ang2 = ang2, ang1
    if is_deg:
        return min(ang2-ang1, ang1+360-ang2)
    else:
        return min(ang2-ang1, ang1+2*math.pi-ang2)

# periodically check FCU's state and takeoff
def takeoff_loop(event):
    global is_takeoff, fcu_state, last_request
    global arming_client, land_client, set_mode_client
    if not is_takeoff:
        if (fcu_state.mode != 'OFFBOARD' and fcu_state.mode != 'AUTO.LOITER') and (rospy.Time.now() - last_request > rospy.Duration(6.0)):
            response = set_mode_client(0, 'OFFBOARD')  #请求解锁
            if response.mode_sent:
                rospy.loginfo('Offboard enabled')
            last_request = rospy.Time.now()
        else:
            if (not fcu_state.armed) and rospy.Time.now() - last_request > rospy.Duration(6.0):
                response = arming_client(True)  #请求起飞
                if response.success:
                    is_takeoff = True
                    rospy.loginfo('Vehicle armed')
                else:
                    rospy.loginfo('Vehicle armed unsuccessfully')
                last_request = rospy.Time.now()
                print('1222222222222222222222')
        print((fcu_state.mode, fcu_state.armed))
    pass

def uav_local_target_loop(event):
    global uav_target_pose_local_pub, uav_local_target, target_lock
    target_lock.acquire()
    uav_target_pose_local_pub.publish(uav_local_target)
    target_lock.release()

def main():
    global gmm_cnt
    global goal_cnt
    global gmm_map
    global target
    global begin_pos, begin_quat
    global resolution, z_const
    global arming_client, land_client, set_mode_client
    global move_pub, goal_pub, uav_target_pose_local_pub
    global tra_total, ang_total
    global is_takeoff
    global fcu_state
    global last_request
    global uav_local_target
    global height
    global target_lock
    global target_deque

    rospy.init_node('gmm_nav', anonymous=True)

    tra_total=0
    ang_total=0
    resolution=0.05
    z_const=0.5
    gmm_cnt=0
    goal_cnt=0
    begin_pos=[0.0, 0.0, 0.0]
    begin_quat=[0.0, 0.0, 0.0, 1.0]
    height=0.25
    is_takeoff=False
    fcu_state=State()
    last_request=rospy.Time.now()
    target_lock=threading.Lock()
    target_deque=deque()

    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    land_client = rospy.ServiceProxy('/mavros/cmd/land', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    goal_pub = rospy.Publisher('/goal_path', Path, queue_size=1)
    uav_target_pose_local_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=100)

    target_sub = rospy.Subscriber('/move_base_simple/goal',PoseStamped, goal_cb,queue_size=100)
    #target_sub = rospy.Subscriber('/move_base_simple/goal_modified',PoseStamped, goal_cb,queue_size=100)
    gmm_sub = rospy.Subscriber('gmm_after_trans',gmm, subgmm_cb,queue_size=100)
    #begin_pose_sub = rospy.Subscriber('/gazebo/model_states',ModelStates, begin_cb,queue_size=100)
    mavpose_sub = rospy.Subscriber('/mavros/local_position/pose',PoseStamped, mavpose_cb,queue_size=100)
    mavros_state_sub = rospy.Subscriber('/mavros/state',State, mavros_state_cb,queue_size=100)

    rate = rospy.Rate(5)
    uav_local_target = PoseStamped()
    uav_local_target.header.seq = 1
    uav_local_target.header.frame_id = 'map'
    uav_local_target.pose.position.z = height
    uav_local_target.pose.orientation.w = 1
    '''
    for i in range(10):
        uav_target_pose_local_pub.publish(uav_local_target)
        rate.sleep()
    '''

    takeoff_timer = rospy.Timer(rospy.Duration(0.25), takeoff_loop)
    uav_local_target_timer = rospy.Timer(rospy.Duration(0.05), uav_local_target_loop)

    while ( 1 ):
        if not is_takeoff:
            print('not ready 2 takeoff')
            rospy.sleep(0.5)
            continue
        print("goal_cnt")
        print(goal_cnt)
        # print("gmm_cnt")
        # print(gmm_cnt)
        if (goal_cnt==0 or gmm_cnt==0):
            print("wait for gmm_map and nav_target!")
            rospy.sleep(0.5)
        else:
            gmm_nav()
            rospy.sleep(0.5)
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
        
    rospy.spin()

if __name__ == "__main__":
    main()
