#!/usr/bin/env python

import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from actionlib_msgs.msg import GoalStatusArray
from controller_manager_msgs.srv import LoadController, SwitchController
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('identification_tests')
    rospy.wait_for_message('move_group/status', GoalStatusArray)
    commander = MoveGroupCommander('panda_arm')
    scene = PlanningSceneInterface(synchronous=True)
    p = PoseStamped()
    p.header.frame_id = commander.get_planning_frame()
    p.pose.position.x = 0.6
    p.pose.position.y = 0.0
    p.pose.position.z = 0.1
    scene.add_box("avoid_area_1", p, (0.8, 0.4, 0.12))
    p.pose.position.x = -0.6
    p.pose.position.y = 0.0
    p.pose.position.z = 0.1
    scene.add_box("avoid_area_2", p, (0.8, 0.4, 0.12))
    p.pose.position.x = 0.0
    p.pose.position.y = 0.6
    p.pose.position.z = 0.1
    scene.add_box("avoid_area_3", p, (2, 0.8, 0.12))
    p.pose.position.x = 0.0
    p.pose.position.y = -0.6
    p.pose.position.z = 0.1
    scene.add_box("avoid_area_4", p, (2, 0.8, 0.12))
    p.pose.position.x = -0.95
    p.pose.position.y = 0
    p.pose.position.z = 1
    scene.add_box("avoid_area_5", p, (0.1, 2, 2))
    rospy.wait_for_service('/controller_manager/load_controller')
    load_controller = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
    load_controller('data_extraction_controller')
    rospy.wait_for_service('/controller_manager/switch_controller')
    switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    start = ['data_extraction_controller']
    #start = []
    stop = []
    switch_controller(start, stop, 2, False, 0)
    rospy.sleep(5)
    start = []
    stop = ['position_joint_trajectory_controller']
    switch_controller(start, stop, 2, False, 0)
    start = []
    stop = ['data_extraction_controller']
    #stop = []
    switch_controller(start, stop, 2, False, 0)
    for i in range(15):
        # move to configuration
        plan_ok = False
        print("\nIteration " + str(i + 1) + "\n")
        while not plan_ok:
            commander.set_random_target()
            plan = commander.plan()
            if plan.joint_trajectory.points:
                print("Planning successful")
                print("\nTarget = " + str(commander.get_joint_value_target()) + "\n")
                plan_ok = True
            else:
                print("Planning unsuccessful")
                print("\nTarget = " + str(commander.get_joint_value_target()) + "\n")
        rospy.wait_for_service('/controller_manager/switch_controller')
        switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        start = ['position_joint_trajectory_controller']
        stop = []
        switch_controller(start, stop, 2, False, 0)
        commander.go(wait=True)
        # recording
        rospy.wait_for_service('/controller_manager/switch_controller')
        switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        start = ['data_extraction_controller']
        #start = []
        stop = []
        switch_controller(start, stop, 2, False, 0)
        rospy.sleep(5)
        start = []
        stop = ['position_joint_trajectory_controller']
        switch_controller(start, stop, 2, False, 0)
        start = []
        stop = ['data_extraction_controller']
        #stop = []
        switch_controller(start, stop, 2, False, 0)
        
    rospy.wait_for_service('/controller_manager/switch_controller')
    switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    start = ['position_joint_trajectory_controller']
    stop = []
    switch_controller(start, stop, 2, False, 0)
    commander = MoveGroupCommander('panda_arm')
    commander.set_named_target('ready')
    plan = commander.plan()
    commander.go(wait=True)
    rospy.wait_for_service('/controller_manager/switch_controller')
    switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    start = ['data_extraction_controller']
    #start = []
    stop = []
    switch_controller(start, stop, 2, False, 0)
    rospy.sleep(5)
    start = []
    stop = ['position_joint_trajectory_controller']
    switch_controller(start, stop, 2, False, 0)
    start = []
    stop = ['data_extraction_controller']
    #stop = []
    switch_controller(start, stop, 2, False, 0)
    
    raw_input("Press enter to stop\n")
