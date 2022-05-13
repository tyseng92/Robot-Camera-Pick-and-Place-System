#!/usr/bin/env python
# By Feb 16, 2021 for ur moveit function testing

import sys
import copy
import rospy
import tf2_ros
import moveit_commander
import moveit_msgs.msg
from copy import deepcopy
import numpy as np
from geometry_msgs.msg import Pose
from tf.transformations import *
from ur_msgs.srv import SetIO

# original lower position
# homept = [-0.0657412146228, 0.174903848331, 0.333372588569, -1, 0, 0, 0]

#homept = [-0.198981608796, 0.189235463563, 0.52755147745, 0.984489155747, -0.00762251609995, 0.174610139566, 0.0153068163191]
homept = [-0.19074840259102496, 0.18646162292055873, 0.49842132473535805, 
            0.9868291923247527, -0.005890241123716808, 0.1611539366305801, 0.012761619946054747]


p1 = [-0.579999906353, 0.357722784762, 0.327568841041, -1, 0, 0, 0]
p2 = [-0.579999906353, 0.257722784762, 0.327568841041, -1, 0, 0, 0]
p3 = [-0.579999906353, 0.157722784762, 0.327568841041, -1, 0, 0, 0]
p4 = [-0.579999906353, 0.057722784762, 0.327568841041, -1, 0, 0, 0]
p5 = [-0.579999906353, -0.057722784762, 0.327568841041, -1, 0, 0, 0]
p6 = [-0.579999906353, -0.157722784762, 0.327568841041, -1, 0, 0, 0]

points = []
points.append(deepcopy(p1))
points.append(deepcopy(p2))
points.append(deepcopy(p3))
points.append(deepcopy(p4))
points.append(deepcopy(p5))
points.append(deepcopy(p6))

if __name__=="__main__":

    rospy.init_node("ws_moveit",anonymous=True)
    rospy.wait_for_service('/ur_hardware_interface/set_io')
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander("manipulator")
    robot = moveit_commander.RobotCommander()
    set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
    i = 0		
    for i in range(1):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(10.0)
        #p0 = group.get_current_pose().pose
        rospy.sleep(3)
        while not rospy.is_shutdown():
            try:
                #trans_ob = tfBuffer.lookup_transform('base_link','charuco',rospy.Time())
                trans_ob = tfBuffer.lookup_transform('base_link','object_1',rospy.Time())
                #trans_rb = tfBuffer.lookup_transform('base_link','ref_link',rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("except")
                rate.sleep()
                continue
            print(trans_ob)
            break
        #p0.position.x = homept[0]
        #p0.position.y = homept[1]
        #p0.position.z = homept[2]
        #p0.orientation.x = homept[3]
        #p0.orientation.y = homept[4]
        #p0.orientation.z = homept[5]
        #p0.orientation.w = homept[6]

        waypoints = []
        f_pos = Pose()
        #print(f_pos, "pose")      
        f_pos.position.x = trans_ob.transform.translation.x
        f_pos.position.y = trans_ob.transform.translation.y
        f_pos.position.z = trans_ob.transform.translation.z + 0.05
        #f_pos.position.z = homept[2]
        f_pos.orientation.x = -1
        f_pos.orientation.y = 0
        f_pos.orientation.z = 0
        f_pos.orientation.w = 0
        waypoints.append(deepcopy(f_pos))

        #f_pos.position.z = trans_ob.transform.translation.z-0.02
        f_pos.position.z = trans_ob.transform.translation.z
        waypoints.append(deepcopy(f_pos))
        (plan,fraction) = group.compute_cartesian_path(waypoints,0.01,0.0)
        group.execute(plan, wait = True)
        
        set_io(fun=1, pin=1, state=1)
        rospy.sleep(1)
        set_io(fun=1, pin=0, state=1)
        rospy.sleep(1)
        ptarget = points[i]
        f_pos.position.z = ptarget[2]
        waypoints.append(deepcopy(f_pos))
        f_pos.position.x = ptarget[0]
        f_pos.position.y = ptarget[1]
        f_pos.position.z = ptarget[2]
        f_pos.orientation.x = ptarget[3]
        f_pos.orientation.y = ptarget[4]
        f_pos.orientation.z = ptarget[5]
        f_pos.orientation.w = ptarget[6]
        waypoints.append(deepcopy(f_pos))
        del waypoints[:2]
        (plan,fraction) = group.compute_cartesian_path(waypoints,0.01,0.0)
        group.execute(plan, wait = True)
        set_io(fun=1, pin=0, state=0)
        rospy.sleep(1)
        set_io(fun=1, pin=1, state=0)
        rospy.sleep(1)
        
        f_pos.position.x = homept[0]
        f_pos.position.y = homept[1]
        f_pos.position.z = homept[2]
        f_pos.orientation.x = homept[3]
        f_pos.orientation.y = homept[4]
        f_pos.orientation.z = homept[5]
        f_pos.orientation.w = homept[6]
        waypoints.append(deepcopy(f_pos))
        del waypoints[:2]
        (plan,fraction) = group.compute_cartesian_path(waypoints,0.01,0.0)
        group.execute(plan, wait = True)
        rospy.loginfo("The ith movement has been done: %s" % i)

    
