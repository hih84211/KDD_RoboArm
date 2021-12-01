#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import threading
import time
import yaml

img = cv2.imread('mother_board.png')
current_pose = [.0, .0, .0]
lock = threading.Lock()
# rospy.Publisher('/KDD_RoboArm/nozzlebase_controller/command', Float64, queue_size = 1),
# rospy.Publisher('/KDD_RoboArm/nozzle_controller/command', Float64, queue_size = 1)]))

class YamlParser:
    parsed_data = {'path': (list, True)}
    
    def __init__(self, inFileName):
        # read YAML
        infile = open(inFileName, 'r')
        ymlcfg = yaml.safe_load(infile)
        infile.close()
        for key in self.parsed_data:
            data = ymlcfg.get(key, None)
            if type(data) != self.parsed_data[key][0]:
                raise Exception('Parameter "{}" has wrong type'.format(key))
            # create attributes on the fly
            setattr(self, key, data)
        
def pixel2coordinate(p_index, val):
    # Gazebo中原點座標與height_map原點間的偏量(update at 12/1)
    x_offset = -485
    y_offset = -123
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
    publishers[0].publish(xyz[0])
    publishers[1].publish(xyz[1])
    # z is temporarily set as a constant 
    publishers[2].publish(-100)
    #publishers[2].publish(xyz[2] - 0.03)
    print('Go to: ', xyz)
    #time.sleep(0.01)
    

def go_for_a_walk(seq, publishers):
    for i in seq:
        val = img[int(i[1])][img.shape[:2][0]-int(i[0])][0]
        coord = pixel2coordinate(i, val)
        goto(coord, publishers)
        while True:
            goto(coord, publishers)
            if r_we_there(coord):
                break

def r_we_there(target):
    tolerance = 1e-8
    err = eucli_dist_sqr(get_position(), target)
    if  err< tolerance:
        print('Arrive, err: ', err)
        return True
    else:
        print('Not even close, err: ',err)
        return False


def get_position():
    global lock
    
    lock.acquire()
    
    global current_pose
    c_pose = [current_pose[0], current_pose[1]]
    print('current: ', current_pose)
    
    lock.release()
    
    return c_pose


def eucli_dist_sqr(p1, p2):
    return (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2


def pose_cb(data):
    lock.acquire()
    
    global current_pose
    #rospy.loginfo(rospy.get_caller_id() +  data.position)
    # print('listener called!!!!!!!!!!!!!!  ', data.position)
    current_pose = [data.position[0], data.position[1], data.position[2]]
    lock.release()


def pose_listener():
    rospy.Subscriber('/KDD_RoboArm/joint_states', JointState, pose_cb)
    rate=rospy.Rate(10)
    

if __name__=='__main__':
    rospy.init_node('command_fromImg')
    x_publisher = rospy.Publisher('/KDD_RoboArm/x_position_controller/command', Float64, queue_size = 1)
    y_publisher = rospy.Publisher('/KDD_RoboArm/y_position_controller/command', Float64, queue_size = 1) 
    z_publisher = rospy.Publisher('/KDD_RoboArm/z_position_controller/command', Float64, queue_size = 1)
    
    t = threading.Thread(target = pose_listener)
    t.start()
    
    # Remember to change the path value 
    cfg = YamlParser('./test.cfg')
    # print(cfg.path)
    path = cfg.path # get a list of x y tuples
    
    print(path)
    
    go_for_a_walk(path, (x_publisher, y_publisher, z_publisher))
    
    t.join()
    
    
