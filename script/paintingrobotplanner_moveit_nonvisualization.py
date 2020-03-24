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

        # mobileplatform.set_goal_joint_tolerance(0.001)
        # mobileplatform.set_max_acceleration_scaling_factor(1)
        # mobileplatform.set_max_velocity_scaling_factor(1)

        rodclimbing_robot = moveit_commander.MoveGroupCommander('rodclimbing_robot')
        # rodclimbing_robot.set_goal_joint_tolerance(0.001)
        # rodclimbing_robot.set_max_acceleration_scaling_factor(1)
        # rodclimbing_robot.set_max_velocity_scaling_factor(1)

        arm = moveit_commander.MoveGroupCommander('aubo5')
        # arm.set_goal_joint_tolerance(0.01)
        # arm.set_max_acceleration_scaling_factor(1)
        # arm.set_max_velocity_scaling_factor(1)

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
        manipulatorendeffector_targetpose_onecell_new=np.zeros(manipulatorendeffector_targetpose_onecell.shape)
        for i in range(len(manipulatorendeffector_targetpose_onecell_new)/2):
            if i%2==0:
                for j in range(len(manipulatorendeffector_targetpose_onecell_new[i])):
                    manipulatorendeffector_targetpose_onecell_new[2*i][j]=manipulatorendeffector_targetpose_onecell[2*i][j]
                    manipulatorendeffector_targetpose_onecell_new[2*i+1][j]=manipulatorendeffector_targetpose_onecell[2*i+1][j]
            else:
                for j in range(len(manipulatorendeffector_targetpose_onecell_new[i])):
                    manipulatorendeffector_targetpose_onecell_new[2*i][j]=manipulatorendeffector_targetpose_onecell[2*i+1][j]
                    manipulatorendeffector_targetpose_onecell_new[2*i+1][j]=manipulatorendeffector_targetpose_onecell[2*i][j]

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
            # for j in range(1):
            if i==0:
                for j in range(1):
                # for j in range(len(manipulatorbase_targetpose[0][i][0])):
                    for k in range(len(manipulatorbase_targetpose[0][i][0][j][0])):
                    # for k in range(len(manipulatorbase_targetpose[0][i][0][j][0])):
                        if k==len(manipulatorbase_targetpose[0][i][0][j][0])-1:
                        # for k in range(len(manipulatorbase_targetpose[0][i][0][j][0])):
                            manipulatorbase_targetpose_onecell = manipulatorbase_targetpose[0][i][0][j][0][k]
                            manipulatorendeffector_targetpose_onecell = manipulatorendeffector_targetpose[0][i][0][j][0][k]

                            # print("manipulatorbase_targetpose_onecell=",manipulatorbase_targetpose_onecell)
                            # print("manipulatorendeffector_targetpose_onecell=",manipulatorendeffector_targetpose_onecell)
                            # Paintrobot.show_obstacles()

                            visualization_num=Paintrobot.motion(manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell,visualization_num)
                            visualization_num=visualization_num+1
    except rospy.ROSInterruptException:
        pass
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)




    # print("manipulatorbase_pose=:", manipulatorbase_pose)
    # print('mobileplatform_joint=:', mobileplatform_joint)
    # print('rodclimbing_robot_joints=:', rodclimbing_robot_joints)

    # def show_obstacles(self):
    #     # 初始化场景对象
    #     scene = PlanningSceneInterface()
    #     # 创建一个发布场景变化信息的发布者
    #     scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
    #
    #     # 创建一个存储物体颜色的字典对象
    #     colors = dict()
    #     # 设置场景物体的名称
    #     table_id = 'table'
    #     box1_id = 'box1'
    #     box2_id = 'box2'
    #     # 设置桌面的高度
    #     table_ground = 0.25
    #     # 设置table、box1和box2的三维尺寸
    #     table_size = [0.2, 0.7, 0.01]
    #     box1_size = [0.1, 0.05, 0.05]
    #     box2_size = [0.05, 0.05, 0.15]
    #
    #     # 将三个物体加入场景当中
    #     table_pose = PoseStamped()
    #     table_pose.header.frame_id = 'base_link'
    #     table_pose.pose.position.x = 0.26
    #     table_pose.pose.position.y = 0.0
    #     table_pose.pose.position.z = table_ground + table_size[2] / 2.0
    #     table_pose.pose.orientation.w = 1.0
    #     scene.add_box(table_id, table_pose, table_size)
    #
    #     box1_pose = PoseStamped()
    #     box1_pose.header.frame_id = 'base_link'
    #     box1_pose.pose.position.x = 0.21
    #     box1_pose.pose.position.y = -0.1
    #     box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
    #     box1_pose.pose.orientation.w = 1.0
    #     scene.add_box(box1_id, box1_pose, box1_size)
    #
    #     box2_pose = PoseStamped()
    #     box2_pose.header.frame_id = 'base_link'
    #     box2_pose.pose.position.x = 0.19
    #     box2_pose.pose.position.y = 0.15
    #     box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
    #     box2_pose.pose.orientation.w = 1.0
    #     scene.add_box(box2_id, box2_pose, box2_size)
    #
    #     # 将桌子设置成红色，两个box设置成橙色
    #     colors[table_id]=self.setColor(0.8, 0, 0, 1.0)
    #     colors[box1_id]=self.setColor(0.8, 0.4, 0, 1.0)
    #     colors[box2_id]=self.setColor(0.8, 0.4, 0, 1.0)
    #     # 初始化规划场景对象
    #     p = PlanningScene()
    #     # 需要设置规划场景是否有差异
    #     p.is_diff = True
    #     # 从颜色字典中取出颜色设置
    #     for color in colors.values():
    #         p.object_colors.append(color)
    #     scene_pub.publish(p)
    #
    # # 设置场景物体的颜色
    # def setColor(self, name, r, g, b, a=0.9):
    #     color = ObjectColor()
    #     color.id = name
    #     color.color.r = r
    #     color.color.g = g
    #     color.color.b = b
    #     color.color.a = a
    #     return color
