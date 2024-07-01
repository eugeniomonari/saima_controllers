#!/usr/bin/env python
import rospy
from controller_manager_msgs.srv import LoadController,SwitchController
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray

move_to_ready = rospy.get_param('/trial_controller_velocity_node/move_to_ready')

if move_to_ready:
    rospy.init_node('move_to_start')
    rospy.wait_for_message('move_group/status', GoalStatusArray)
    commander = MoveGroupCommander('panda_arm')
    commander.set_named_target('ready')
    plan = commander.plan()
    commander.go(wait=True)

rospy.wait_for_service('/controller_manager/load_controller')
load_controller = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
load_controller('trial_controller_velocity')

raw_input("Press enter to start the controller\n")

rospy.wait_for_service('/controller_manager/switch_controller')
switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
start = ['trial_controller_velocity']
if move_to_ready:
    stop = ['position_joint_trajectory_controller']
else:
    stop = []
start_asap = False
timeout = 0
switch_controller(start, stop, 2, start_asap, timeout)

raw_input("Press enter to stop the contoller and save the data\n")

rospy.wait_for_service('/controller_manager/switch_controller')
switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
start = []
stop = ['trial_controller_velocity']
switch_controller(start, stop, 2, start_asap, timeout)
