#!/usr/bin/env python

###########################
# Author: Yi Herng Ong
# Lab: Oregon State University Grasping Lab Group
# Date: 12/14/2018
# Purpose : This script can compute final transformation of endeffector and visualize the arm movement in OpenRave for experiment setup of human study
############################

from openravepy import *
import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from tf import transformations as tr
import math, os, sys
import time
from openravepy import ikfast
from tf import transformations as tr
import pdb
# General function for robot 
class Robot(object):
	def __init__(self, robot): # Define robot 
		self.env = Environment()
		if robot == "barrettwam":
			self.env.Load("/home/graspinglab/openrave/src/robots/barrettwam.robot.xml")
			self.robot = self.env.GetBodies()[0]
			robot_trans = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.306],[0,0,0,1]])
			self.robot.SetTransform(np.matmul(robot_trans, self.robot.GetTransform()))
			# print self.robot.GetTransform()

		else:
			sys.exit("invalid robot name")
		self.env.SetViewer('qtcoin')
		self.manip = self.robot.SetActiveManipulator('arm')
		# print self.robot.GetActiveDOFLimits()

		

	def load_scene(self):
		self.env.Load("/home/graspinglab/openrave/src/data/table.kinbody.xml")
		self.scene = self.env.GetBodies()[1]
		self.scene.SetTransform(np.array([[1,0,0,0], [0,1,0,0.654], [0,0,1,0],[0,0,0,1]]))

	def load_object(self):
		self.env.Load("/home/graspinglab/Control_Strategy_Gen/cube_h8_w8_e8.stl")
		self.object = self.env.GetBodies()[2]
		print self.object.GetTransform()
		obj_trans = np.array([[1,0,0,0], [0,1,0,0.654], [0,0,1,0.055],[0,0,0,1]]) # weird transform, recheck openrave api
		self.object.SetTransform(np.matmul(obj_trans, self.object.GetTransform())) # transform might be off, recheck the code
		# self.object.SetTransform(np.array([[ 1,0,0,-0.0056],[0,1,0,1.02291],[0,0,1,0.072],[0,0,0,1]])) # transform might be off, recheck the code

	# Simple planner that moves robot to goal transformation, input can be either pose transformation or joint angles
	def move_robot(self, goal):
		basemanip = interfaces.BaseManipulation(self.robot) 
		if len(goal) == 4: # pose transformation
			try:
				move = basemanip.MoveToHandPosition(matrices=[goal],outputtrajobj=True) # call motion planner with goal pose transformation
			except:
				sys.exit("Transformation pose is not working, choose another pose")	
		if len(goal) == 7: # joint angles
			try:
				move = basemanip.MoveManipulator(goal=goal,outputtrajobj=True) # call motion planner with goal joint angles
			except:
				sys.exit("Joint angle pose is not working, choose another pose")	
		traj = []
		for i in range(move.GetNumWaypoints()):
			temp_traj = move.GetWaypoint(i)
			traj.append(temp_traj[0:7]) # joint 1 to 7 of WAM
		return np.array(traj)
		
		# sol = self.manip.FindIKSolution(goal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution


	def set_goal(self):
		# tgoal = np.array([[1,0,0,0],[0,math.cos(-1.5708),-math.sin(-1.5708),1],[0,math.sin(-1.5708),math.cos(-1.5708),0.07],[0,0,0,1]])
		# rot_gen = np.array([[math.cos(-0.523599), 0, math.sin(-0.523599), 0], [math.sin(-1.5708)*math.cos(-0.523599), -math.cos(-1.5708), -math.sin(-1.5708)*math.cos(-0.523599),1],[-math.cos(-1.5708)*math.sin(-0.523599), math.sin(-1.5708), math.cos(-1.5708)*math.cos(-0.523599), 1],[0,0,0,1]])
		# print rot_gen

		rotx = np.array([[1,0,0,0],[0,math.cos(-1.5708),-math.sin(-1.5708),0],[0,math.sin(-1.5708),math.cos(-1.5708),0],[0,0,0,1]]) 
		roty = np.array([[math.cos(-0.436332),0,math.sin(-0.436332),0],[0,1,0,0],[-math.sin(-0.436332),0,math.cos(-0.436332),0],[0,0,0,1]]) 
		rotz = np.array([[math.cos(-0.436332), -math.sin(-0.436332), 0, 0], [math.sin(-0.436332), math.cos(-0.436332), 0,0], [0,0,1,0], [0,0,0,1]])
		rot = np.matmul(rotx, roty)
		# trans = np.array([[1,0,0,0.05],[0,1,0,0.67691],[0,0,1,0.075],[0,0,0,1]]) # base translation
		trans = np.array([[1,0,0,0],[0,1,0,0.7],[0,0,1,0.15],[0,0,0,1]]) # base translation
		
		tgoal = np.matmul(trans, rotx)
		traj = self.move_robot(tgoal)
		f = open("yposneg5.csv","w")
		for i in range(len(tgoal)):
			for j in range(len(traj[0])):
				f.write(str(traj[i,j]))
				f.write(",")
			f.write("\n")
		f.close()
		# obj_trans = np.array([[1,0,0,0], [0,1,0,-0.1], [0,0,1,0],[0,0,0,1]]) # weird transform, recheck openrave api
		# self.object.SetTransform(np.matmul(obj_trans, self.object.GetTransform()))
		print self.robot.GetLink("wam7").GetTransform()


	# This function computes valid grasps in the current environment and robot configuration
	# Note that it might take awhile
	def find_Grasps(self, num_grasps): # num_grasps is number of grasps you would like to take
		print "in here"
		gmodel = databases.grasping.GraspingModel(self.robot, self.object)
		if not gmodel.load():
			gmodel.autogenerate()
		validgrasps, validindices = gmodel.computeValidGrasps(returnnum=num_grasps)
		grasps = []
		for i in range(len(validgrasps)):
			grasps.append(gmodel.getGlobalGraspTransform(validgrasps[i], collisionfree=True))
		return np.array(grasps)

	# Origin pose : tgoal = np.array([[1,0,0,0],[0,math.cos(-1.5708),-math.sin(-1.5708),1],[0,math.sin(-1.5708),math.cos(-1.5708),0.07],[0,0,0,1]])
	def sampling_in_xyz(self, tgoal):
		x = np.array([0.02, 0.04, 0.06, -0.02, -0.04, -0.06])
		y = np.array([-0.02, 0.04, 0.06, 0.08])
		z = np.array([-0.02, 0.04, 0.06, 0.08])
		
		x_samples = []
		# sampling x
		for i in x:
			temp = tgoal
			temp[0,3] = temp[0,3] + i
			x_samples.append(temp)

		#sampling y
		y_samples = []
		for j in y:
			tempy = tgoal
			tempy[1,3] = tempy[1,3] + j
			y_samples.append(tempy)
		# sampling z
		z_samples = []
		for k in z:
			tempz = tgoal
			tempz[2,3] = tempz[2,3] + k
			z_samples.append(tempz)

	def sampling_rpw(self, tgoal):
		r = np.array([-math.radians(45), -math.radians(30), -math.radians(15), math.radians(15), math.radians(30), math.radians(45)])
		p = np.array([-math.radians(15), math.radians(15), math.radians(30), math.radians(45)])
		w = np.array([-math.radians(15), -math.radians(30), math.radians(15), math.radians(30)])
		
if __name__ == '__main__':
	r = Robot("barrettwam")
	r.load_scene()
	# r.load_object()
	# r.set_goal()
	# grasps = r.find_Grasps(1)
	# r.move_robot(grasps[0])