#!/usr/bin/env python
# By Feb 04, 2022 for ur moveit function testing

import sys
import copy
import rospy
import tf2_ros
import moveit_commander
import moveit_msgs.msg
from copy import deepcopy
import numpy as np

# For SUTD high position
#homept = [-0.198981608796, 0.189235463563, 0.52755147745, 0.984489155747, -0.00762251609995, 0.174610139566, 0.0153068163191]
homept = [-0.19074840259102496, 0.18646162292055873, 0.49842132473535805, 
            0.9868291923247527, -0.005890241123716808, 0.1611539366305801, 0.012761619946054747]

if __name__=="__main__":

	rospy.init_node("ws_moveit1",anonymous=True)
	moveit_commander.roscpp_initialize(sys.argv)
	group = moveit_commander.MoveGroupCommander("manipulator")
	pt = group.get_current_pose()
	posem = group.get_current_pose().pose
	robot = moveit_commander.RobotCommander()
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
	i = 0		
	waypoints = []
	ptarget = homept
	posem.position.z = ptarget[2]
	waypoints.append(deepcopy(posem))
	posem.position.x = ptarget[0]
	posem.position.y = ptarget[1]
	#posem.position.z = ptarget[2]
	posem.orientation.x = ptarget[3]
	posem.orientation.y = ptarget[4]
	posem.orientation.z = ptarget[5]
	posem.orientation.w = ptarget[6]
	waypoints.append(deepcopy(posem))
	(plan,fraction) = group.compute_cartesian_path(waypoints,0.01,0.0)
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(plan)
	display_trajectory_publisher.publish(display_trajectory);
	group.execute(plan, wait = True)
	posem = group.get_current_pose().pose
	print(posem)
	rospy.loginfo("The ith movement has been done: %s" % "pick")
