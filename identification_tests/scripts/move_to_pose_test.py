#!/usr/bin/env python

import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from actionlib_msgs.msg import GoalStatusArray
from controller_manager_msgs.srv import LoadController, SwitchController
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    # move to configuration
    rospy.init_node('move_to_pose')
    rospy.wait_for_message('move_group/status', GoalStatusArray)
    commander = MoveGroupCommander('panda_arm')
    scene = PlanningSceneInterface(synchronous=True)
    p = PoseStamped()
    p.header.frame_id = commander.get_planning_frame()
    p.pose.position.x = 0.6
    p.pose.position.y = 0.0
    p.pose.position.z = 0.1
    scene.add_box("avoid_area_1", p, (0.8, 0.4, 0.2))
    p.pose.position.x = -0.6
    p.pose.position.y = 0.0
    p.pose.position.z = 0.1
    scene.add_box("avoid_area_2", p, (0.8, 0.4, 0.2))
    p.pose.position.x = 0.0
    p.pose.position.y = 0.6
    p.pose.position.z = 0.1
    scene.add_box("avoid_area_3", p, (2, 0.8, 0.2))
    p.pose.position.x = 0.0
    p.pose.position.y = -0.6
    p.pose.position.z = 0.1
    scene.add_box("avoid_area_4", p, (2, 0.8, 0.2))
    plan_ok = False
    while not plan_ok:
        #commander.set_random_target()
        commander.set_named_target('ready')
        #set_target = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, -0.8630]
        commander.set_joint_value_target(set_target)
        plan = commander.plan()
        # set_target = commander.get_joint_value_target()
        # last_planned = plan.joint_trajectory.points[-1].positions
        # print(last_planned)
        # condition = True
        # for i in range(7):
        #     if abs(set_target[i] - last_planned[i]) > 0.05:
        #         condition = False
        if plan.joint_trajectory.points:
        # if plan.joint_trajectory.points:  # True if trajectory contains points
            print("Planning successful")
            print(plan.joint_trajectory.points[-1].positions)
            print(commander.get_joint_value_target())
            plan_ok = True
        else:
            print("Planning unsuccessful")
    commander.go(wait=True)

    # recording
    rospy.wait_for_service('/controller_manager/load_controller')
    load_controller = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
    load_controller('data_extraction')
    rospy.wait_for_service('/controller_manager/switch_controller')
    switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    start = ['data_extraction']
    stop = []
    switch_controller(start, stop, 2, False, 0)
    rospy.sleep(5)
    start = []
    stop = ['position_joint_trajectory_controller']
    switch_controller(start, stop, 2, False, 0)
    start = []
    stop = ['data_extraction']
    switch_controller(start, stop, 2, False, 0)
    raw_input("Press enter to stop\n")
