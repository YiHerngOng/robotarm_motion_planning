#!/usr/bin/env python

import rospy
import moveit_commander
import sys, os
import moveit_msgs.msg
import tf.transformations
import std_msgs.msg
import geometry_msgs.msg





if __name__ == '__main__':
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node("barrett_wam_move_group", anonymous=True)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group_name = "barrett_wam"
	move_group = moveit_commander.MoveGroupCommander("arm")
	display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
	# print robot.get_current_state()
	pose_goal = geometry_msgs.msg.Pose()
	# pose_goal.orientation.w = 3.93951372334e-05
	# pose_goal.position.x = -0.436436190034
	# pose_goal.position.y = 0.324093850516
	# pose_goal.position.z = 2.22897663575
	pose_goal.position.x = 0.05
	pose_goal.position.y = 0.67691
	pose_goal.position.z = 0.075
	# print move_group.get_current_pose()
	# pose_goal = [0.05,0.67691,0.075,1.5708,0,0]
	move_group.set_pose_target(pose_goal)
	plan = move_group.go(wait=True)
	move_group.stop()
	move_group.clear_pose_targets()
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(plan)
	# print display_trajectory.trajectory
	# # # Publish
	display_trajectory_publisher.publish(display_trajectory)
	# pose: 
	# position: 
	#   x: -0.0454255068706
	#   y: -0.26627327155
	#   z: 0.983911269951
	# orientation: 
	#   x: 1.74457892726e-05
	#   y: -9.27151541767e-05
	#   z: 1.66856125495e-05
	#   w: 0.999999995411
