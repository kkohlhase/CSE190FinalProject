#!/usr/bin/env python

# mdp implementation needs to go here

import rospy
from read_config import read_config
import numpy
from cse_190_assi_3.msg import *
from copy import deepcopy

class Mdp():

	def __init__(self):
		self.config = read_config()

		self.stepCost = self.config['reward_for_each_step']
		self.wallCost = self.config['reward_for_hitting_wall']
		self.goalCost = self.config['reward_for_reaching_goal']
		self.pitCost = self.config['reward_for_falling_in_pit']

		self.probMoveCorrect = self.config['prob_move_forward']
		self.probMoveBackwards = self.config['prob_move_backward']
		self.probMoveLeftOfIntended = self.config['prob_move_left']
		self.probMoveRightOfIntended = self.config['prob_move_right']

		self.discountFactor = self.config['discount_factor']

		self.maxIterations = self.config['max_iterations']
		self.thresholdDifference = self.config['threshold_difference']

		self.numRows = self.config['map_size'][0]
		self.numCols = self.config['map_size'][1]

		self.start = self.config['start'] #[2,0] for self.config1 - will always be 1D array... 1 coord
		self.goal = self.config['goal'] #[0,3] for self.config1 - will always be 1D array
		self.walls = self.config['walls'] #[1,1] for self.config1, will be a 2d array for self.configs 2/3...multiple coords
		self.pits = self.config['pits'] #[1,3] for self.config1, will be a 2d array for self.configs 2/3

		self.numPossibleMoves = len(self.config['move_list']) #should be 4 in all self.configs

		self.originalGrid =[[0 for x in range(self.numCols)] for y in range(self.numRows)]
		self.rewardGrid =[[0 for x in range(self.numCols)] for y in range(self.numRows)]
		self.policyGrid =[["" for x in range(self.numCols)] for y in range(self.numRows)]


		self.policyList_publisher = rospy.Publisher(
			"/results/policy_list",
			PolicyList,
			queue_size=10)

		#set up grid
		self.populateGrids()

		for k in range(0, self.maxIterations):
			#calculate the first value iteration
			self.newRewardGrid = deepcopy(self.rewardGrid)
			for i in range(0, self.numRows):
				for j in range(0, self.numCols):
					self.policyGrid[i][j] = self.calculateMaxVStar(i, j)

			self.policyGrid1D = []
			for row in self.policyGrid:
				for col in row:
					self.policyGrid1D.append(col)

			policyList = PolicyList()
			policyList.data = self.policyGrid1D
			rospy.sleep(1)
			self.policyList_publisher.publish(policyList)

			sumOfDifferences = 0
			for m in range(0, self.numRows):
				for n in range(0, self.numCols):
					sumOfDifferences = sumOfDifferences + abs(self.newRewardGrid[m][n] - self.rewardGrid[m][n])

			if sumOfDifferences < self.thresholdDifference:
				return

			self.rewardGrid = deepcopy(self.newRewardGrid)

			print "finished updating policy", k, self.policyGrid1D, self.rewardGrid, self.newRewardGrid


		#publish

		#update original grid with new values

		#repeat until convergence or reached maximum number of iterations

	def calculateVStar(self,x,y, direction):
		#calculate previous values
		hitEdgeOfMapCorrect = False
		hitEdgeOfMapBackward = False
		hitEdgeOfMapLeft = False
		hitEdgeOfMapRight = False
		if(direction == 'N'):
			correctX = x-1
			if correctX < 0:
				correctX = correctX + 1
				hitEdgeOfMapCorrect = True
			correctY = y
			backwardX = x+1
			if backwardX >= self.numRows:
				backwardX = backwardX - 1
				hitEdgeOfMapBackward = True
			backwardY = y
			leftX = x
			leftY = y-1
			if leftY < 0:
				leftY = leftY + 1
				hitEdgeOfMapLeft = True
			rightX = x
			rightY = y+1
			if rightY >= self.numCols:
				rightY = rightY - 1
				hitEdgeOfMapRight = True
		elif(direction == 'S'):
			correctX = x+1
			if correctX >= self.numRows:
				correctX = correctX - 1
				hitEdgeOfMapCorrect = True
			correctY = y
			backwardX = x-1
			if backwardX < 0:
				backwardX = backwardX + 1
				hitEdgeOfMapBackward = True
			backwardY = y
			leftX = x
			leftY = y+1
			if leftY >= self.numCols:
				leftY = leftY - 1
				hitEdgeOfMapLeft = True
			rightX = x
			rightY = y-1
			if rightY < 0:
				rightY = rightY + 1
				hitEdgeOfMapRight = True
		elif(direction == 'W'):
			correctX = x
			correctY = y-1
			if correctY < 0:
				correctY = correctY + 1
				hitEdgeOfMapCorrect = True
			backwardX = x
			backwardY = y+1
			if backwardY >= self.numCols:
				backwardY = backwardY - 1
				hitEdgeOfMapBackward = True
			leftX = x+1
			if leftX >= self.numRows:
				leftX = leftX - 1
				hitEdgeOfMapLeft = True
			leftY = y
			rightX = x-1
			if rightX < 0:
				rightX = rightX + 1
				hitEdgeOfMapRight = True
			rightY = y
		elif(direction == 'E'):
			correctX = x
			correctY = y+1
			if correctY >= self.numCols:
				correctY = correctY - 1
				hitEdgeOfMapCorrect = True
			backwardX = x
			backwardY = y-1
			if backwardY < 0:
				backwardY = backwardY + 1
				hitEdgeOfMapBackward = True
			leftX = x-1
			if leftX < 0:
				leftX = leftX + 1
				hitEdgeOfMapLeft = True
			leftY = y
			rightX = x+1
			if rightX >= self.numRows:
				rightX = rightX - 1
				hitEdgeOfMapRight = True
			rightY = y
		else:
			print 'ERROR something wrong with direction'

		if self.originalGrid[correctX][correctY] == -1:
			correctX = x
			correctY = y
			hitEdgeOfMapCorrect = True

		if self.originalGrid[backwardX][backwardY] == -1:
			backwardX = x
			backwardY = y
			hitEdgeOfMapBackward = True

		if self.originalGrid[leftX][leftY] == -1:
			leftX = x
			leftY = y
			hitEdgeOfMapLeft = True

		if self.originalGrid[rightX][rightY] == -1:
			rightX = x
			rightY = y
			hitEdgeOfMapRight = True

		VStarMoveCorrect = self.probMoveCorrect * (self.calculateReward(correctX, correctY, hitEdgeOfMapCorrect) + (self.discountFactor*self.rewardGrid[correctX][correctY]))
		VStarMoveBackwards = self.probMoveBackwards * (self.calculateReward(backwardX, backwardY, hitEdgeOfMapBackward) + (self.discountFactor*self.rewardGrid[backwardX][backwardY]))
		VStarMoveLeft = self.probMoveLeftOfIntended * (self.calculateReward(leftX, leftY, hitEdgeOfMapLeft) + (self.discountFactor*self.rewardGrid[leftX][leftY]))
		VStarMoveRight = self.probMoveRightOfIntended * (self.calculateReward(rightX, rightY, hitEdgeOfMapRight) + (self.discountFactor*self.rewardGrid[rightX][rightY]))

		VStar = VStarMoveCorrect + VStarMoveBackwards + VStarMoveLeft + VStarMoveRight

		return VStar

		#need to check if it is in the grid or if it is a wall/pit/goal


	def calculateMaxVStar(self,x,y):
		VstarArray = [0 for v in range (4)]
		direction = ["" for z in range(4)]
		direction[0] = "N"
		direction[1] = "S"
		direction[2] = "W"
		direction[3] = "E"

		if self.originalGrid[x][y] == -1:
			return "WALL"
		if self.originalGrid[x][y] == -10:
			return "PIT"
		if self.originalGrid[x][y] == 10:
			return "GOAL"

		for i in range(0, self.numPossibleMoves):
			VstarArray[i] = self.calculateVStar(x,y, direction[i])

		maxVStar = numpy.amax(VstarArray)

		self.newRewardGrid[x][y] = maxVStar

		if maxVStar == VstarArray[0]:
			return direction[0]

		if maxVStar == VstarArray[1]:
			return direction[1]

		if maxVStar == VstarArray[2]:
			return direction[2]

		if maxVStar == VstarArray[3]:
			return direction[3]


		

	def populateGrids(self):
		#0 is normal space, 10 is goal, -1 is obstacle, -10 is pit
		
		self.originalGrid[self.goal[0]][self.goal[1]] = 10
		self.rewardGrid[self.goal[0]][self.goal[1]] = self.goalCost

		for x,y in self.walls:
			self.originalGrid[x][y] = -1
			self.rewardGrid[x][y] = self.wallCost

		for w,z in self.pits:
			self.originalGrid[w][z] = -10
			self.rewardGrid[w][z] = self.pitCost


	def calculateReward(self, x, y, hitEdgeOfMap):
		reward = 0

		if self.originalGrid[x][y] == -1  or hitEdgeOfMap:
			reward = reward + self.wallCost
		elif self.originalGrid[x][y] == -10:
			reward = reward + self.pitCost + self.stepCost
		elif self.originalGrid[x][y] == 10:
			reward = reward + self.goalCost + self.stepCost
		elif self.originalGrid[x][y] == 0:
			reward = reward + self.stepCost
		else:
			print "ERROR with rewards"

		return reward


