#!/usr/bin/env python3

import math
import os
import select
import sys
import xml.dom.minidom
import rospy
from std_msgs.msg import Float64
if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios

STAGES = 3
LIN_VEL_STEP_SIZE = 0.0005
LIN_VEL_MAX = 0.005

msg = """
Control Your cnc_test!
---------------------------
Moving around:

w/s : moving along x-axis
a/d : moving along y-axis
q/e : moving along z-axis
x   : stop moving


CTRL-C to quit
"""

e = """
Communications Failed
"""


class SoSmoothSimulator:
    def __init__(self, robot):
        self.joint_list = []
        self.free_joints = {}
        self.joint_vel = [0, 0, 0]
        self.joint_map = {}
        self.dependent_joints = get_param("dependent_joints", {})
        self.use_mimic = get_param("use_mimic_tags", True)
        self.use_small = get_param("use_smallest_joint_limits", True)
        self.zeros = get_param("zeros")
        self.pub_def_positions = get_param("publish_default_positions", True)
        self.pub_def_vels = get_param("publish_default_velocities", False)
        self.pub_def_efforts = get_param("publish_default_efforts", False)

        self.init_urdf(robot)
        for name in self.joint_list:
            if name not in self.free_joints:
                continue
            joint = self.free_joints[name]

            if joint['min'] == joint['max']:
                continue

            self.joint_map[name] = joint  # joints' name and their details

        self.publishers = dict(zip(self.joint_list,
                                   [rospy.Publisher('/KDD_RoboArm/x_position_controller/command', Float64, queue_size=1),
                                    rospy.Publisher('/KDD_RoboArm/y_position_controller/command', Float64, queue_size=1),
                                    rospy.Publisher('/KDD_RoboArm/z_position_controller/command', Float64, queue_size=1),]))

    def init_urdf(self, robot):
        robot = robot.getElementsByTagName('robot')[0]
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype in ['fixed', 'floating', 'planar']:
                    continue
                name = child.getAttribute('name')
                self.joint_list.append(name)
                if jtype == 'continuous':
                    minval = -math.pi
                    maxval = math.pi
                else:
                    try:
                        limit = child.getElementsByTagName('limit')[0]
                        minval = float(limit.getAttribute('lower'))
                        maxval = float(limit.getAttribute('upper'))
                    except:
                        rospy.logwarn("%s is not fixed, nor continuous, but limits are not specified!" % name)
                        continue

                safety_tags = child.getElementsByTagName('safety_controller')
                if self.use_small and len(safety_tags) == 1:
                    tag = safety_tags[0]
                    if tag.hasAttribute('soft_lower_limit'):
                        minval = max(minval, float(tag.getAttribute('soft_lower_limit')))
                    if tag.hasAttribute('soft_upper_limit'):
                        maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))

                mimic_tags = child.getElementsByTagName('mimic')
                if self.use_mimic and len(mimic_tags) == 1:
                    tag = mimic_tags[0]
                    entry = {'parent': tag.getAttribute('joint')}
                    if tag.hasAttribute('multiplier'):
                        entry['factor'] = float(tag.getAttribute('multiplier'))
                    if tag.hasAttribute('offset'):
                        entry['offset'] = float(tag.getAttribute('offset'))

                    self.dependent_joints[name] = entry
                    continue
                if name in self.dependent_joints:
                    continue
                if self.zeros and name in self.zeros:
                    zeroval = self.zeros[name]
                elif minval > 0 or maxval < 0:
                    zeroval = (maxval + minval) / 2
                else:
                    zeroval = 0

                joint = {'min': minval, 'max': maxval, 'zero': zeroval, 'velocity': 0}
                if self.pub_def_positions:
                    joint['position'] = zeroval
                if self.pub_def_vels:
                    joint['velocity'] = 0.0
                if self.pub_def_efforts:
                    joint['effort'] = 0.0
                if jtype == 'continuous':
                    joint['continuous'] = True
                self.free_joints[name] = joint
        return True

    def make_simple_profile(self, output, input, slop):
        if input > output:
            output = min(input, output + slop)
        elif input < output:
            output = max(input, output - slop)
        else:
            output = input

        return output

    def update(self, k):
        if k == 'w':
            joint_name = 'xy_joint'
            current_lin_vel = self.joint_map[joint_name]['velocity']
            target_lin_vel = constrain(current_lin_vel + LIN_VEL_STEP_SIZE, -LIN_VEL_MAX, LIN_VEL_MAX)
            self.joint_map[joint_name]['velocity'] = target_lin_vel
            print('y-axis vel: ', target_lin_vel)
        elif k == 's':
            joint_name = 'xy_joint'
            current_lin_vel = self.joint_map[joint_name]['velocity']
            target_lin_vel = constrain(current_lin_vel - LIN_VEL_STEP_SIZE, -LIN_VEL_MAX, LIN_VEL_MAX)
            self.joint_map[joint_name]['velocity'] = target_lin_vel
            print('y-axis vel: ', target_lin_vel)
        elif k == 'a':
            joint_name = 'bx_joint'
            current_lin_vel = self.joint_map[joint_name]['velocity']
            target_lin_vel = constrain(current_lin_vel + LIN_VEL_STEP_SIZE, -LIN_VEL_MAX, LIN_VEL_MAX)
            self.joint_map[joint_name]['velocity'] = target_lin_vel
            print('x-axis vel: ', target_lin_vel)
        elif k == 'd':
            joint_name = 'bx_joint'
            current_lin_vel = self.joint_map[joint_name]['velocity']
            target_lin_vel = constrain(current_lin_vel - LIN_VEL_STEP_SIZE, -LIN_VEL_MAX, LIN_VEL_MAX)
            self.joint_map[joint_name]['velocity'] = target_lin_vel
            print('x-axis vel: ', target_lin_vel)
        elif k == ' ' or k == 'x':
            for name, joint in self.joint_map.items():
                joint['velocity'] = 0


def get_key():
    if os.name == 'nt':
        if sys.version_info[0] >= 3:
            return msvcrt.getch().decode()
        else:
            return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = None

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value


def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input


if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    joints_lin_vel = [0, 0, 0]

    rospy.init_node('cnc_snake_control')

    path = open('/home/peter/PycharmProjects/cnc_snake_control/cnc_test17.urdf', mode='r', encoding='utf-8-sig').read()
    robot = xml.dom.minidom.parseString(path)
    smooth_controller = SoSmoothSimulator(robot)

    status = 0
    target_linear_vel = 0.0
    control_linear_vel = 0.
    rate = rospy.Rate(40)

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = get_key()
            if key:
                smooth_controller.update(key)
            for name, joint in smooth_controller.joint_map.items():
                joint['position'] = constrain(joint['position'] + joint['velocity'], joint['min'], joint['max'])
                smooth_controller.publishers[name].publish(joint['position'])
            rate.sleep()

    except Exception as ex:
        print(ex.with_traceback())

    '''finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)'''

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
