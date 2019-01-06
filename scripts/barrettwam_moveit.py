#!/usr/bin/env python

import rospy
import moveit_commander
import sys, os
import moveit_msgs.msg
import tf.transformations
import std_msgs.msg
import geometry_msgs.msg
import pdb
import tf, math



if __name__ == '__main__':
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node("barrett_wam_move_group", anonymous=True)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group_name = "barrett_wam"
	move_group = moveit_commander.MoveGroupCommander("arm")
	move_group.set_planner_id("RRTConnectkConfigDefault")
	display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
	# print robot.get_current_state()
	pose_goal = geometry_msgs.msg.Pose()
	# pose_goal.orientation.w = 3.93951372334e-05
	# print move_group.get_current_pose()
	rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.0)
	quat = tf.transformations.quaternion_from_euler(0, math.radians(90), 0)
	pose_goal.position.x = 0.852060061486
	pose_goal.position.y = 0.01302963661748
	pose_goal.position.z = 0.691377158031
	# pose_goal.orientation.x = -0.723902057684
	# pose_goal.orientation.y = 6.19094989285e-05
	# pose_goal.orientation.z = -0.689902741946
	# pose_goal.orientation.w = 0.000117062679053
	pose_goal.orientation.x = quat[0]
	pose_goal.orientation.y = quat[1]
	pose_goal.orientation.z = quat[2]
	pose_goal.orientation.w = quat[3]	
	move_group.set_pose_target(pose_goal)
	plan = move_group.plan()
	move_group.set_planning_time(10)
	move_group.go(wait=True)
	move_group.execute(plan, wait=True)
	move_group.stop()
	# move_group.clear_pose_targets()
	# display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	# display_trajectory.trajectory_start = robot.get_current_state()
	# display_trajectory.trajectory.append(plan)
	# print display_trajectory.trajectory
	# # # Publish
	# display_trajectory_publisher.publish(display_trajectory)
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
