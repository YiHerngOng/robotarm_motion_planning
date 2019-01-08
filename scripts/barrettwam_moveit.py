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

# base pose : position [0.6972, 0, 0.8] orientation [0, 90, 0]
# vary poses based on the base pose. 

class robot(object):
	def __init__(self, robot_name):
		# Initialize Moveit interface
		moveit_commander.roscpp_initialize(sys.argv)
		if robot_name == "barrett_wam":
			rospy.init_node("barrett_wam_move_group", anonymous=True)
			self.robot = moveit_commander.RobotCommander()
			self.scene = moveit_commander.PlanningSceneInterface()
			self.group = moveit_commander.MoveGroupCommander()
			rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.0)
			rospy.sleep(2)
			self.disp_traj = moveit_msgs.smg.DisplayTrajectory()
			self.disp_traj.trajectory_start = self.robot.get_current_state()

	def planner_type(self):
		self.group.set_planner_id("RRTConnectkConfigDefault")

	def move_to_Goal(self,ee_pose):
		pose_goal = geometry_msgs.msg.Pose()
		quat = tf.transformations.quaternion_from_euler(ee_pose[3], math.radians(ee_pose[4]), ee_pose[5])
		pose_goal.position.x = ee_pose[0]
		pose_goal.position.y = ee_pose[1]
		pose_goal.position.z = ee_pose[2]
		pose_goal.orientation.x = quat[0]
		pose_goal.orientation.y = quat[1]
		pose_goal.orientation.z = quat[2]
		pose_goal.orientation.w = quat[3]	
		self.group.set_pose_target(pose_goal)
		self.plan = self.group.plan()
		self.group.set_planning_time(10)
		self.group.go(wait=True)
		self.group.execute(self.plan, wait=True)
		self.group.stop()
		self.group.clear_pose_targets()
		rospy.sleep(2)

	def display_Trajectory(self):
		self.disp_traj_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
		self.disp_traj.trajectory.append(self.plan)
		self.disp_traj_publisher.publish(self.disp_traj)

	def sampling_Poses(self):
		base_pose = [0.6972,0,0.82,0,90,0]
		# sampling x (y axis in rviz)
		x_poses = []
		for i in range(6):
			temp = base_pose[:]
			if i < 3:
				temp[1] += 0.02 
			else:
				temp[1] -= 0.02
			x_poses.append(temp)

		# sampling y (z axis in rviz)
		y_poses = []
		for j in range(4):
			temp = base_pose[:]
			if j == 0:
				temp[2] -= 0.02
			else:
				temp[2] += 0.02
			y_poses.append(temp)

		# sampling z (x_axis in rviz)
		z_poses = []
		for k in range(4):
			temp = base_pose[:]
			temp[0] += 0.02
			z_poses.append(temp)

		r_poses = []
		for l in range(6):
			temp = base_pose[:]
			if l <= 2:
				temp[3] -= 15
			if l > 2:
				temp[3] += 15
			r_poses.append(temp)

		p_poses = []
		for m in range(4):
			temp = base_pose[:]
			if m == 0:
				temp[4] -= 15
			else:
				temp[4] += 15
			p_poses.append(temp)

		w_poses = []
		for n in range(4):
			temp = base_pose[:]
			if n <= 1:
				temp[5] -= 15
			if n > 1:
				temp[5] += 15
			w_poses.append(temp)








if __name__ == '__main__':
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node("barrett_wam_move_group", anonymous=True)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group_name = "barrett_wam"
	move_group = moveit_commander.MoveGroupCommander("arm")
	move_group.set_planner_id("RRTConnectkConfigDefault")
	display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	# print robot.get_current_state()
	pose_goal = geometry_msgs.msg.Pose()
	# print move_group.get_current_pose()
	rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.0)
	rospy.sleep(2)
	quat = tf.transformations.quaternion_from_euler(0, math.radians(90), 0)
	pose_goal.position.x = 0.6972
	pose_goal.position.y = 0.0
	pose_goal.position.z = 0.80
	pose_goal.orientation.x = quat[0]
	pose_goal.orientation.y = quat[1]
	pose_goal.orientation.z = quat[2]
	pose_goal.orientation.w = quat[3]
	move_group.set_pose_target(pose_goal)
	plan = move_group.plan()
	move_group.set_planning_time(5)
	move_group.go(wait=True)
	move_group.execute(plan, wait=True)
	move_group.stop()
	move_group.clear_pose_targets()
	display_trajectory.trajectory.append(plan)
	print plan
	# # Publish
	display_trajectory_publisher.publish(display_trajectory)
