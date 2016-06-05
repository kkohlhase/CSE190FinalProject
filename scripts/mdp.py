#!/usr/bin/env python

# mdp implementation needs to go here

import rospy
from read_config import read_config
import numpy
from cse_190_assi_3.msg import *
from copy import deepcopy
import pprint

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
		self.probMoveUpOfIntended = self.config['prob_move_up']
		self.probMoveDownOfIntended = self.config['prob_move_down']

		self.discountFactor = self.config['discount_factor']

		self.maxIterations = self.config['max_iterations']
		self.thresholdDifference = self.config['threshold_difference']

		self.numRows = self.config['map_size'][0]
		self.numCols = self.config['map_size'][1]
		self.numHeight = self.config['map_size'][2]

		self.start = self.config['start'] #[2,0,0] for self.config1 - 
		self.goal = self.config['goal'] #[0,3,2] for self.config1 - 
		print self.goal[0], self.goal[1], self.goal[2]
		self.walls = self.config['walls'] #[1,1,0][2,0,1][0,3,1] for self.config1,
		self.pits = self.config['pits'] #[1,3,0] for self.config1, 

		self.numPossibleMoves = len(self.config['move_list']) #should be 6 in all self.configs

		self.originalGrid =[[[0 for x in range(self.numHeight)] for y in range(self.numCols)] for z in range(self.numRows)]
		pprint.pprint(self.originalGrid)
		self.rewardGrid =[[[0 for x in range(self.numHeight)] for y in range(self.numCols)] for z in range(self.numRows)]
		self.policyGrid =[[["" for x in range(self.numHeight)] for y in range(self.numCols)] for z in range(self.numRows)]


		self.policyList_publisher = rospy.Publisher(
			"/results/policy_list",
			PolicyList,
			queue_size=10)

		#set up grid
		print 'Starting MDP'
		self.populateGrids()

		for l in range(0, self.maxIterations):
			#calculate the first value iteration
			self.newRewardGrid = deepcopy(self.rewardGrid)
			for i in range(0, self.numRows):
				for j in range(0, self.numCols):
					for k in range(0, self.numHeight):
						self.policyGrid[i][j][k] = self.calculateMaxVStar(i, j, k)

			self.policyGrid1D = []
			for row in self.policyGrid:
				for col in row:
					for height in col:
						self.policyGrid1D.append(height)

			policyList = PolicyList()
			policyList.data = self.policyGrid1D
			rospy.sleep(1)
			self.policyList_publisher.publish(policyList)

			sumOfDifferences = 0
			for m in range(0, self.numRows):
				for n in range(0, self.numCols):
					for o in range(0, self.numHeight):
						sumOfDifferences = sumOfDifferences + abs(self.newRewardGrid[m][n][o] - self.rewardGrid[m][n][o])

			if sumOfDifferences < self.thresholdDifference:
				return

			self.rewardGrid = deepcopy(self.newRewardGrid)

			print "finished updating policy", l, self.policyGrid1D

		pprint.pprint(self.policyGrid)
		#publish

		#update original grid with new values

		#repeat until convergence or reached maximum number of iterations

	def calculateVStar(self,x,y,z, direction):  #just this left to edit
		#calculate previous values
		hitEdgeOfMapForward = False
		hitEdgeOfMapBackward = False
		hitEdgeOfMapLeft = False
		hitEdgeOfMapRight = False
		hitEdgeOfMapUp = False
		hitEdgeOfMapDown = False
		if(direction == 'F'):
			forwardX = x-1
			if forwardX < 0:
				forwardX = forwardX + 1
				hitEdgeOfMapForward = True
			forwardY = y
			forwardZ = z
			backwardX = x+1
			if backwardX >= self.numRows:
				backwardX = backwardX - 1
				hitEdgeOfMapBackward = True
			backwardY = y
			backwardZ = z
			leftX = x
			leftY = y-1
			leftZ = z
			if leftY < 0:
				leftY = leftY + 1
				hitEdgeOfMapLeft = True
			rightX = x
			rightY = y+1
			rightZ = z
			if rightY >= self.numCols:
				rightY = rightY - 1
				hitEdgeOfMapRight = True
			upX = x
			upY = y
			upZ = z+1
			if upZ >= self.numHeight:
				upZ = upZ - 1
				hitEdgeOfMapUp = True
			downX = x
			downY = y
			downZ = z-1
			if downZ < 0:
				downZ = downZ + 1
				hitEdgeOfMapDown = True
		elif(direction == 'B'):
			forwardX = x+1
			if forwardX >= self.numRows:
				forwardX = forwardX - 1
				hitEdgeOfMapForward = True
			forwardY = y
			forwardZ = z
			backwardX = x-1
			if backwardX < 0:
				backwardX = backwardX + 1
				hitEdgeOfMapBackward = True
			backwardY = y
			backwardZ = z
			leftX = x
			leftY = y+1
			leftZ = z
			if leftY >= self.numCols:
				leftY = leftY - 1
				hitEdgeOfMapLeft = True
			rightX = x
			rightY = y-1
			rightZ = z
			if rightY < 0:
				rightY = rightY + 1
				hitEdgeOfMapRight = True
			upX = x
			upY = y
			upZ = z+1
			if upZ >= self.numHeight:
				upZ = upZ - 1
				hitEdgeOfMapUp = True
			downX = x
			downY = y
			downZ = z-1
			if downZ < 0:
				downZ = downZ + 1
				hitEdgeOfMapDown = True
		elif(direction == 'L'):
			forwardX = x
			forwardY = y-1
			forwardZ = z
			if forwardY < 0:
				forwardY = forwardY + 1
				hitEdgeOfMapForward = True
			backwardX = x
			backwardY = y+1
			backwardZ = z
			if backwardY >= self.numCols:
				backwardY = backwardY - 1
				hitEdgeOfMapBackward = True
			leftX = x+1
			if leftX >= self.numRows:
				leftX = leftX - 1
				hitEdgeOfMapLeft = True
			leftY = y
			leftZ = z
			rightX = x-1
			if rightX < 0:
				rightX = rightX + 1
				hitEdgeOfMapRight = True
			rightY = y
			rightZ = z
			upX = x
			upY = y
			upZ = z+1
			if upZ >= self.numHeight:
				upZ = upZ - 1
				hitEdgeOfMapUp = True
			downX = x
			downY = y
			downZ = z-1
			if downZ < 0:
				downZ = downZ + 1
				hitEdgeOfMapDown = True
		elif(direction == 'R'):
			forwardX = x
			forwardY = y+1
			forwardZ = z
			if forwardY >= self.numCols:
				forwardY = forwardY - 1
				hitEdgeOfMapForward = True
			backwardX = x
			backwardY = y-1
			backwardZ = z
			if backwardY < 0:
				backwardY = backwardY + 1
				hitEdgeOfMapBackward = True
			leftX = x-1
			if leftX < 0:
				leftX = leftX + 1
				hitEdgeOfMapLeft = True
			leftY = y
			leftZ = z
			rightX = x+1
			if rightX >= self.numRows:
				rightX = rightX - 1
				hitEdgeOfMapRight = True
			rightY = y
			rightZ = z
			upX = x
			upY = y
			upZ = z+1
			if upZ >= self.numHeight:
				upZ = upZ - 1
				hitEdgeOfMapUp = True
			downX = x
			downY = y
			downZ = z-1
			if downZ < 0:
				downZ = downZ + 1
				hitEdgeOfMapDown = True
		elif(direction == 'U'):
			forwardX = x
			forwardY = y
			forwardZ = z+1
			if forwardZ >= self.numHeight:
				forwardZ = forwardZ - 1
				hitEdgeOfMapUp = True
			backwardX = x
			backwardY = y
			backwardZ = z-1
			if backwardZ < 0:
				backwardZ = backwardZ + 1
				hitEdgeOfMapDown = True
			leftX = x
			leftY = y-1
			leftZ = z
			if leftY < 0:
				leftY = leftY + 1
				hitEdgeOfMapLeft = True
			rightX = x
			rightY = y+1
			rightZ = z
			if rightY >= self.numCols:
				rightY = rightY - 1
				hitEdgeOfMapRight = True
			upX = x+1
			upY = y
			upZ = z
			if upX >= self.numRows:
				upX = upX - 1
				hitEdgeOfMapUp = True
			downX = x-1
			downY = y
			downZ = z
			if downX < 0:
				downX = downX + 1
				hitEdgeOfMapDown = True
		elif(direction == 'D'):
			forwardX = x
			forwardY = y
			forwardZ = z-1
			if forwardZ < 0:
				forwardZ = forwardZ + 1
				hitEdgeOfMapUp = True
			backwardX = x
			backwardY = y
			backwardZ = z+1
			if backwardZ >= self.numHeight:
				backwardZ = backwardZ - 1
				hitEdgeOfMapDown = True
			leftX = x
			leftY = y-1
			leftZ = z
			if leftY < 0:
				leftY = leftY + 1
				hitEdgeOfMapLeft = True
			rightX = x
			rightY = y+1
			rightZ = z
			if rightY >= self.numCols:
				rightY = rightY - 1
				hitEdgeOfMapRight = True
			upX = x-1
			upY = y
			upZ = z
			if upX < 0:
				upX = upX + 1
				hitEdgeOfMapUp = True
			downX = x+1
			downY = y
			downZ = z
			if downX >= self.numRows:
				downX = downX - 1
				hitEdgeOfMapDown = True
		else:
			print 'ERROR something wrong with direction'

		if self.originalGrid[forwardX][forwardY][forwardZ] == -1:
			forwardX = x
			forwardY = y
			forwardZ = z
			hitEdgeOfMapForward = True

		if self.originalGrid[backwardX][backwardY][backwardZ] == -1:
			backwardX = x
			backwardY = y
			backwardZ = z
			hitEdgeOfMapBackward = True

		if self.originalGrid[leftX][leftY][leftZ] == -1:
			leftX = x
			leftY = y
			leftZ = z
			hitEdgeOfMapLeft = True

		if self.originalGrid[rightX][rightY][rightZ] == -1:
			rightX = x
			rightY = y
			rightZ = z
			hitEdgeOfMapRight = True

		if self.originalGrid[upX][upY][upZ] == -1:
			upX = x
			upY = y
			upZ = z
			hitEdgeOfMapUp = True

		if self.originalGrid[downX][downY][downZ] == -1:
			downX = x
			downY = y
			downZ = z
			hitEdgeOfMapDown = True

		VStarMoveForward = self.probMoveCorrect * (self.calculateReward(forwardX, forwardY, forwardZ, hitEdgeOfMapForward) + (self.discountFactor*self.rewardGrid[forwardX][forwardY][forwardZ]))
		VStarMoveBackwards = self.probMoveBackwards * (self.calculateReward(backwardX, backwardY, backwardZ, hitEdgeOfMapBackward) + (self.discountFactor*self.rewardGrid[backwardX][backwardY][backwardZ]))
		VStarMoveLeft = self.probMoveLeftOfIntended * (self.calculateReward(leftX, leftY, leftZ, hitEdgeOfMapLeft) + (self.discountFactor*self.rewardGrid[leftX][leftY][leftZ]))
		VStarMoveRight = self.probMoveRightOfIntended * (self.calculateReward(rightX, rightY, rightZ, hitEdgeOfMapRight) + (self.discountFactor*self.rewardGrid[rightX][rightY][rightZ]))
		VStarMoveUp = self.probMoveUpOfIntended * (self.calculateReward(upX, upY, upZ, hitEdgeOfMapUp) + (self.discountFactor*self.rewardGrid[upX][upY][upZ]))
		VStarMoveDown = self.probMoveDownOfIntended * (self.calculateReward(downX, downY, downZ, hitEdgeOfMapDown) + (self.discountFactor*self.rewardGrid[downX][downY][downZ]))

		VStar = VStarMoveForward + VStarMoveBackwards + VStarMoveLeft + VStarMoveRight + VStarMoveUp + VStarMoveDown

		return VStar

		#need to check if it is in the grid or if it is a wall/pit/goal


	def calculateMaxVStar(self,x,y,z):
		VstarArray = [0 for e in range (6)]
		direction = ["" for f in range(6)]
		direction[0] = "U" #up
		direction[1] = "D" #down
		direction[2] = "F" #forward
		direction[3] = "B" #backward
		direction[4] = "L" #left
		direction[5] = "R" #right

		if self.originalGrid[x][y][z] == -1:
			return "WALL"
		if self.originalGrid[x][y][z] == -10:
			return "PIT"
		if self.originalGrid[x][y][z] == 10:
			return "GOAL"

		for i in range(0, self.numPossibleMoves):
			VstarArray[i] = self.calculateVStar(x,y,z, direction[i])

		maxVStar = numpy.amax(VstarArray)

		self.newRewardGrid[x][y][z] = maxVStar

		if maxVStar == VstarArray[0]:
			return direction[0]

		if maxVStar == VstarArray[1]:
			return direction[1]

		if maxVStar == VstarArray[2]:
			return direction[2]

		if maxVStar == VstarArray[3]:
			return direction[3]

		if maxVStar == VstarArray[4]:
			return direction[4]

		if maxVStar == VstarArray[5]:
			return direction[5]


		

	def populateGrids(self):
		#0 is normal space, 10 is goal, -1 is obstacle, -10 is pit
		
		self.originalGrid[self.goal[0]][self.goal[1]][self.goal[2]] = 10
		self.rewardGrid[self.goal[0]][self.goal[1]][self.goal[2]] = self.goalCost

		for x,y,z in self.walls:
			self.originalGrid[x][y][z] = -1
			self.rewardGrid[x][y][z] = self.wallCost

		for a,b,c in self.pits:
			self.originalGrid[a][b][c] = -10
			self.rewardGrid[a][b][c] = self.pitCost


	def calculateReward(self, x, y, z, hitEdgeOfMap):
		reward = 0

		if self.originalGrid[x][y][z] == -1  or hitEdgeOfMap:
			reward = reward + self.wallCost
		elif self.originalGrid[x][y][z] == -10:
			reward = reward + self.pitCost + self.stepCost
		elif self.originalGrid[x][y][z] == 10:
			reward = reward + self.goalCost + self.stepCost
		elif self.originalGrid[x][y][z] == 0:
			reward = reward + self.stepCost
		else:
			print "ERROR with rewards"

		return reward


