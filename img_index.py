#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math
import listofpathpoint
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import threading
import time

img = cv2.imread('path to photo')
current_pose = [.0, .0, .0]
# rospy.Publisher('/KDD_RoboArm/nozzlebase_controller/command', Float64, queue_size = 1),
# rospy.Publisher('/KDD_RoboArm/nozzle_controller/command', Float64, queue_size = 1)]))

def on_EVENT_LBUTTONDOWN( x, y, param):
    xy = (x, img.shape[:2][0]-y)
    print('index: ', xy)
    val = img[y][x][0]
    print('val: ', val)
    coord = pixel2coordinate(xy, val)
    print('3D coordinate: ', coord)
    goto(coord, param)
    '''val = img[y][x][0]
    print('val: ', val)
    coord = pixel2coordinate(xy, val)
    print('3D coordinate: ', coord)
    param[0].publish(coord[0])
    param[1].publish(coord[1])
    param[2].publish(coord[2] - 0.03)
    cv2.circle(img, (x, y), 1, (255, 0, 0), thickness=-1)
    cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                1.0, (0, 0, 0), thickness=1)
    img.imshow('image', img)'''

def pixel2coordinate(p_index, val):
    # Gazebo中原點座標與height_map原點間的偏量
    x_offset = -460
    y_offset = -120
    # height_map 中的 0 與真實值之間的偏量
    z_bias = -9.7    
    tf_x = 0.239144/1024
    tf_y = 0.128838/554
    tf_z = 0.014156/110
    
    x = (p_index[0] + x_offset) * tf_x
    y = (p_index[1] + y_offset) * tf_y
    z = (val+z_bias)*tf_z
    
    return(x, y, z)
    
def goto(xyz, publishers):
    global current_pose
    publishers[0].publish(xyz[0])
    publishers[1].publish(xyz[1])
    publishers[2].publish(xyz[2] - 0.03)
    print('current: ', current_pose)

def go_for_a_walk(seq, publishers):
    for i in seq:
        val = img[i[0]][i[1]][0]
        coord = pixel2coordinate(i, val)
        while True:
            goto(coord, publishers)
            if r_we_there(coord):
                break
    
def r_we_there(target):
    tolerance = 1e-4
    if eucli_dist_sqr(get_position(), target) < tolerance:
        return True
    else:
        return False

def get_position():
    global current_pose
    return [current_pose[0], current_pose[1]]

def eucli_dist_sqr(p1, p2):
    return (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2


def pose_cb(data):
    global current_pose
    #rospy.loginfo(rospy.get_caller_id() +  data.position)
    # print('listener called!!!!!!!!!!!!!!  ', data.position)
    current_pose = [data.position[0], data.position[1], data.position[2]]

def pose_listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.Subscriber('/KDD_RoboArm/joint_states', JointState, pose_cb)
    rate=rospy.Rate(10)
    # spin() simply keeps python from exiting until this node is stopped
    
    
def job():
  for i in range(150):
    print("Child thread:", i)
    time.sleep(1)


if __name__=='__main__':
    rospy.init_node('command_fromImg')
    x_publisher = rospy.Publisher('/KDD_RoboArm/x_position_controller/command', Float64, queue_size = 1)
    y_publisher = rospy.Publisher('/KDD_RoboArm/y_position_controller/command', Float64, queue_size = 1) 
    z_publisher = rospy.Publisher('/KDD_RoboArm/z_position_controller/command', Float64, queue_size = 1)
    
    path = listofpathpoint.main() # get a list of x y tuples
    for point in path:
        on_EVENT_LBUTTONDOWN(point[0],point[1],(x_publisher, y_publisher, z_publisher))
    t = threading.Thread(target = pose_listener)
    t.start()
    while(True):
        try:
            
            cv2.waitKey(100)
        except Exception:
            # 等待 t 這個子執行緒結束
            t.join()
            break
    # 等待 t 這個子執行緒結束
    t.join()
