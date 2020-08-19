#!/usr/bin/env python

# obtain robot pose from sensor information


import rospy
import math
import tf
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from transfer import *
from aubo_kinematics import *
from Quaternion import *

class robot_pose_computation():
    def __init__(self):
        self.T1=np.eye(4)
        self.T2=np.eye(4)
        self.T3=np.eye(4)
        self.wholerobot_pose=np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.aubo_pose_sub = rospy.Subscriber('aubo_pose',Pose, manipulator_computation)

    # part one: mobile platform frame to map frame
    def tf_computation(self):
        try:
            (mobile_trans, mobile_quaternion) = listener.lookupTransform('/base_link', '/aubo_baselink', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        # obtain trans
        mobile_trans
        # quternion to euler

        # final: obtain T1 and z-yaw angle

    # part two: aubo manipulator base to mobile platform frame: height/angle is obtained from the linear/rotation encoder respectively
    def rodclimbing_computation(self):
        constantvalue=0.5
        rotation_length=0.6
        linear_encoder_length=rospy.get_param("")
        rotation_encoder_angle=rospy.get_param("")
        tran_x=rotation_length*math.cos(rotation_encoder_angle)
        tran_y=rotation_length*math.sin(rotation_encoder_angle)
        tran_z=linear_encoder_length+constantvalue

        rot_mat=rotz(rotation_encoder_angle)
        # obtain T2 and z-yaw angle


    # part three: aubo manipulator end effector to base frame
    def manipulator_computation(self,data):
        p = np.array([data.position.x, data.position.y, data.position.z])
        q = np.array([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
        # obtain T3 and rpy angles
        mat_generation = pose2mat()
        self.T3 = mat_generation.mat4x4(p, q)

    def wholerobot_computation(self):
        wholerobot_T=np.array(self.T1*self.T2*self.T3)
        tran_x=wholerobot_T[0][3]
        tran_y=wholerobot_T[1][3]
        tran_z=wholerobot_T[2][3]

        # obtain rpy angles and T matrix

if __name__ == '__main__':
    rospy.init_node('endeffectorpose_computation', anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        rate.sleep()

