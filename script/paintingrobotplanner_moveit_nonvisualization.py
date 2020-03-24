#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from math import *
import numpy as np
import numpy.matlib
import moveit_commander
import scipy.io as io
from transfer import *
from aubo_kinematics import *
from Quaternion import *

from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA

from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose


class Renovationrobot_motion():
    def __init__(self):
        self.parameterx=0.430725381079
        self.parametery=-0.00033063639818
        self.parameterz=1.32061628232
        self.distance=0.10

    def homing(self):
        # definition of three moveit groups
        mobileplatform = moveit_commander.MoveGroupCommander('mobileplatform')
        rodclimbing_robot= moveit_commander.MoveGroupCommander('rodclimbing_robot')
        arm = moveit_commander.MoveGroupCommander('aubo5')
        # these groups moves to initial poses
        mobileplatform.set_named_target('home1')
        mobileplatform.go()
        rospy.sleep(1)

        rodclimbing_robot.set_named_target('home2')
        rodclimbing_robot.go()
        rospy.sleep(1)

        arm.set_named_target('home3')
        arm.go()
        rospy.sleep(1)
    def motion(self,manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell,visualization_num):
        marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

        # definition of three moveit groups
        mobileplatform = moveit_commander.MoveGroupCommander('mobileplatform')
        rodclimbing_robot = moveit_commander.MoveGroupCommander('rodclimbing_robot')
        arm = moveit_commander.MoveGroupCommander('aubo5')

        # computation of target joints of mobile platform
        theta_z=manipulatorbase_targetpose_onecell[0][5]
        deltax=self.parameterx*cos(theta_z)-self.parametery*sin(theta_z)
        deltay=self.parameterx*sin(theta_z)+self.parametery*cos(theta_z)
        mobileplatform_targetjoints=[manipulatorbase_targetpose_onecell[0][0]-deltax,-(manipulatorbase_targetpose_onecell[0][1]-deltay),theta_z]

        # computation of mobile platform collision-avoidance path
        # input: the present position of mobile platform
        # input: the next position of mobile platform
        # output: collision-avoidance path between the present position and next position

        # motion of mobile platform
        mobileplatform.set_joint_value_target(mobileplatform_targetjoints)
        mobileplatform_state=mobileplatform.go()
        rospy.sleep(0.2)

        # computation of target joints of rodclimbing_robot
        manipulatorbase_pose = rodclimbing_robot.get_current_pose('aubo_baselink').pose
        mobileplatform_joint = mobileplatform.get_current_joint_values()
        rodclimbing_robot_joints = rodclimbing_robot.get_current_joint_values()
        rodclimbing_robot_targetjoints=[manipulatorbase_targetpose_onecell[0][2]-manipulatorbase_pose.position.z+rodclimbing_robot_joints[0],rodclimbing_robot_joints[1]]

        # motion of rod climbing robot
        print("rodclimbing_robot_targetjoints=",rodclimbing_robot_targetjoints)

        rodclimbing_robot.set_joint_value_target(rodclimbing_robot_targetjoints)
        rodclimbing_robot_state=rodclimbing_robot.go()
        rospy.sleep(0.2)


        # computation of forward paths of manipulator
        manipulatorendeffector_targetpose_onecell_new=manipulatorendeffector_targetpose_onecell

        # computation of inverse joints of manipulator
        interval = 0.10
        aubo_joints_list=np.array([0.7432146906113353, -0.6072259915236845, 1.387201205355398, 1.9944271968790823, 0.8275816361835613, 1.5707963267948966])
        previous_aubo_joints=aubo_joints_list
        for i in range(len(manipulatorendeffector_targetpose_onecell_new)-1):
            p1=np.array([manipulatorendeffector_targetpose_onecell_new[i][0],manipulatorendeffector_targetpose_onecell_new[i][1],manipulatorendeffector_targetpose_onecell_new[i][2]])
            p2=np.array([manipulatorendeffector_targetpose_onecell_new[i+1][0],manipulatorendeffector_targetpose_onecell_new[i+1][1],manipulatorendeffector_targetpose_onecell_new[i+1][2]])
            distance=sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2+(p1[2]-p2[2])**2)
            p=np.zeros(3)
            if i==len(manipulatorendeffector_targetpose_onecell_new)-2:
                num = int(distance / interval)+2
            else:
                num = int(distance / interval)+1
            for j in range(num):
                p[0] = p1[0] + (p2[0] - p1[0]) / distance * interval * j
                p[1] = p1[1] + (p2[1] - p1[1]) / distance * interval * j
                p[2] = p1[2] + (p2[2] - p1[2]) / distance * interval * j
                q=np.array([manipulatorendeffector_targetpose_onecell_new[i][3],manipulatorendeffector_targetpose_onecell_new[i][4],manipulatorendeffector_targetpose_onecell_new[i][5]])
                T_mat_generation=pose2mat()
                mat=T_mat_generation.mat4x4(p,q)
                mat1=np.ravel(mat)
                mat2=mat1.tolist()
                aubo_arm = Aubo_kinematics()
                aubo_joints_onepoint=aubo_arm.GetInverseResult(mat2,previous_aubo_joints)
                previous_aubo_joints=aubo_joints_onepoint
                aubo_joints_list=np.append(aubo_joints_list,aubo_joints_onepoint,axis=0)

        points_num=len(aubo_joints_list)/6
        if mobileplatform_state==True and rodclimbing_robot_state==True:
            for i in range(points_num):
                # motion of manipulator
                aubo_joints=np.array(aubo_joints_list[6*i:6*i+6])
                print('aubo_joints=:',aubo_joints)
                arm.set_joint_value_target(aubo_joints)
                arm_state=arm.go()
        #     rospy.sleep(0.2)

        # homing of manipulator
        arm.set_named_target('home3')
        arm.go()
        visualization_num = visualization_num + 1
        return visualization_num


if __name__ == "__main__":
    mat_path="/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_planning/script/planner_result/data.mat"
    # mat_path="package://paintingrobot_planning/script/data.mat"
    data = io.loadmat(mat_path)
    manipulatorbase_targetpose=data['renovation_cells_manipulatorbase_positions']
    manipulatorendeffector_targetpose=data['manipulator_endeffector_positions']

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('paintingrobot_simulation', anonymous=True)
    Paintrobot = Renovationrobot_motion()
    # Paintrobot.homing()
    visualization_num=1
    try:
        # for i in range(len(manipulatorbase_targetpose[0])):
        #     for j in range(len(manipulatorbase_targetpose[0][i][0])):
        #         for k in range(len(manipulatorbase_targetpose[0][i][0][j][0])):
        # for i in range(len(manipulatorbase_targetpose[0])):
        for i in range(1):
            for j in range(1):
                for k in range(1):
                    manipulatorbase_targetpose_onecell = manipulatorbase_targetpose[0][i][0][j][0][k]
                    manipulatorendeffector_targetpose_onecell = manipulatorendeffector_targetpose[0][i][0][j][0][k]

                    visualization_num=Paintrobot.motion(manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell,visualization_num)
                    visualization_num=visualization_num+1
    except rospy.ROSInterruptException:
        pass
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)



