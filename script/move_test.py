#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def main() :
    pos_1 = [-0.04483205476869756, -0.733215634022848, -2.0507729689227503, -1.6661017576800745, 1.5725396871566772, -0.004635636006490529]

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface' , anonymous = True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory, queue_size = 10)

    print("Initalize Robot Pose...")
    joint_goal = [-0.34350759187807256, -0.9218924681292933, -2.0065024534808558, -1.522297207509176, 1.63822603225708, -0.2967131773578089]
    move_group.go(joint_goal , wait = True)

    print("Is Robot can move freely ? Safe is important !")
    inp = raw_input("Go ? y/n :")[0]

    if inp == 'y':
        joint_goal = pos_1
        move_group.go(joint_goal , wait = True)
        move_group.stop()


if __name__ == "__main__" :
    main()
