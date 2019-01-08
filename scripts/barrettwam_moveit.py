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
			self.group = moveit_commander.MoveGroupCommander("arm")
			rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.0)
			rospy.sleep(2)
			self.disp_traj = moveit_msgs.msg.DisplayTrajectory()
			self.disp_traj.trajectory_start = self.robot.get_current_state()
			self.base_pose = [0.6972,0,0.82,0,90,0] 
	def planner_type(self):
		self.group.set_planner_id("RRTConnectkConfigDefault")

	def move_to_Goal(self,ee_pose):
		pose_goal = geometry_msgs.msg.Pose()
		quat = tf.transformations.quaternion_from_euler(math.radians(ee_pose[3]), math.radians(ee_pose[4]), math.radians(ee_pose[5]))
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

# base_pose = [0.6972,0,0.82,0,90,0] 
base_pose = [0.6972,0,0.86,0,90,0] 


def sampling_x_Poses(increment, num): # for x 
	# sampling x (y axis in rviz)
	global base_pose

	poses = []
	temp = base_pose[:]
	for i in range(num/2):
		temp[1] -= increment
		temp1 = temp[:]
		poses.append(temp1)

	temp = base_pose[:]
	for j in range(num/2):
		temp[1] += increment
		temp1 = temp[:]
		poses.append(temp1)
	return poses

def sampling_y_Poses(increment, num): # for y
	global base_pose
	poses = [] 
	temp = base_pose[:]
	temp[2] -= increment
	temp1 = temp[:]
	poses.append(temp1)
	temp = base_pose[:]
	for i in range(num-1):
		temp[2] += increment
		temp1 = temp[:]
		poses.append(temp1)
	return poses

def sampling_z_Poses(increment, num): # for z
	poses = []
	global base_pose
	temp = base_pose[:]
	for i in range(num):
		temp[0] -= increment
		temp1 = temp[:]
		poses.append(temp1)
	return poses

def sampling_rw_Poses(increment, num, axis): # for roll and yaw
	global base_pose
	if axis == "r":
		position = 3
	# if axis == "w":
	# 	position = 5
	poses = []
	temp = base_pose[:]
	for i in range(num/2):
		temp[3] -= increment
		temp[5] -= increment
		temp1 = temp[:]
		poses.append(temp1)
	temp = base_pose[:]
	for j in range(num/2):
		temp[3] += increment
		temp[5] += increment
		temp1 = temp[:]
		poses.append(temp1)
	return poses

def sampling_p_Poses(increment, num): # for pitch only
	global base_pose
	poses = []
	temp = base_pose[:]
	temp[4] -= increment
	temp1 = temp[:]
	poses.append(temp1)
	# pdb.set_trace()
	temp = base_pose[:]
	for n in range(num-1):
		temp[4] += increment
		temp1 = temp[:]
		poses.append(temp1)
		# pdb.set_trace()
	return poses



if __name__ == '__main__':
	x = sampling_x_Poses(0.02, 6)
	y = sampling_y_Poses(0.02, 4)
	z = sampling_z_Poses(0.02, 4)

	r = sampling_rw_Poses(15, 6, "r")
	# w = sampling_rw_Poses(15, 4, "w")
	# p = sampling_p_Poses(15, 4)

	Robot = robot("barrett_wam")
	Robot.planner_type()
	Robot.move_to_Goal(base_pose)
	# for i in range(len(x)):
	# 	rospy.loginfo("Planning and executing x_{}".format(i))
	# 	Robot.move_to_Goal(x[i])
	rospy.sleep(2)
	# for j in range(len(y)):
	# 	rospy.loginfo("Planning and executing y_{}".format(j))
	# 	rospy.loginfo("Current Y pose {}".format(y[j]))
	# 	Robot.move_to_Goal(y[j])	

	# for k in range(len(z)):
	# 	rospy.loginfo("Planning and executing z_{}".format(k))
	# 	Robot.move_to_Goal(z[k])

	for l in range(len(r)):
		rospy.loginfo("Planning and executing r_{}".format(l))
		Robot.move_to_Goal(r[l])

	# for m in range(len(p)):
	# 	rospy.loginfo("Planning and executing p_{}".format(m))
	# 	Robot.move_to_Goal(p[m])

	# for n in range(len(w)):
	# 	rospy.loginfo("Planning and executing w_{}".format(n))
	# 	Robot.move_to_Goal(w[n])
	# pdb.set_trace()

	# moveit_commander.roscpp_initialize(sys.argv)
	# rospy.init_node("barrett_wam_move_group", anonymous=True)
	# robot = moveit_commander.RobotCommander()
	# scene = moveit_commander.PlanningSceneInterface()
	# group_name = "barrett_wam"
	# move_group = moveit_commander.MoveGroupCommander("arm")
	# move_group.set_planner_id("RRTConnectkConfigDefault")
	# display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
	# display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	# display_trajectory.trajectory_start = robot.get_current_state()
	# # print robot.get_current_state()
	# pose_goal = geometry_msgs.msg.Pose()
	# # print move_group.get_current_pose()
	# rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.0)
	# rospy.sleep(2)
	# quat = tf.transformations.quaternion_from_euler(0, math.radians(90), 0)
	# pose_goal.position.x = 0.6972
	# pose_goal.position.y = 0.0
	# pose_goal.position.z = 0.80
	# pose_goal.orientation.x = quat[0]
	# pose_goal.orientation.y = quat[1]
	# pose_goal.orientation.z = quat[2]
	# pose_goal.orientation.w = quat[3]
	# move_group.set_pose_target(pose_goal)
	# plan = move_group.plan()
	# move_group.set_planning_time(5)
	# move_group.go(wait=True)
	# move_group.execute(plan, wait=True)
	# move_group.stop()
	# move_group.clear_pose_targets()
	# display_trajectory.trajectory.append(plan)
	# print plan
	# # # Publish
	# display_trajectory_publisher.publish(display_trajectory)
