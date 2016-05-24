#!/usr/bin/env python

# mdp implementation needs to go here

import rospy
from read_config import read_config
import numpy

config = read_config()

stepCost = config['reward_for_each_step']

probMoveCorrect = config['prob_move_forward']
probMoveBackwards = config['prob_move_backward']
probMoveLeftOfIntended = config['prob_move_left']
probMoveRightOfIntended = config['prob_move_right']

discountFactor = config['discount_factor']

maxIterations = config['max_iterations']
thresholdDifference = config['threshold_difference']

numRows = config['map_size'][0]
numCols = config['map_size'][1]

start = config['start'] #[2,0] for config1 - will always be 1D array... 1 coord
goal = config['goal'] #[0,3] for config1 - will always be 1D array
walls = config['wall'] #[1,1] for config1, will be a 2d array for configs 2/3...multiple coords
pits = config['pit'] #[1,3] for config1, will be a 2d array for configs 2/3

numPossibleMoves = len(config[move_list]) #should be 4 in all configs

def MDPFindPolicies():

	#calculate the first value iteration
	for i in range(0, numRows):
		for j in range(0, numCols):
			calculateMaxVStar(i, j)

	#publish

	#update original grid with new values

	#repeat until convergence or reached maximum number of iterations

def calculateVStar(x,y, direction):
	#calculate previous values
	if(direction == 'up'):
		correctX = x-1
		correctY = y
		backwardX = x+1
		backwardY = y
		leftX = x
		leftY = y-1
		rightX = x
		rightY = y+1
	else if(direction == 'down'):
		correctX = x+1
		correctY = y
		backwardX = x-1
		backwardY = y
		leftX = x
		leftY = y+1
		rightX = x
		rightY = y-1
	else if(direction == 'left'):
		correctX = x
		correctY = y-1
		backwardX = x
		backwardY = y+1
		leftX = x+1
		leftY = y
		rightX = x-1
		rightY = y
	else if(direction == 'right'):
		correctX = x
		correctY = y+1
		backwardX = x
		backwardY = y-1
		leftX = x-1
		leftY = y
		rightX = x+1
		rightY = y
	else:
		print 'ERROR something wrong with direction'


	VStarMoveCorrect = probMoveCorrect * (stepCost + (discountFactor*previousValue[correctX][correctY]))
	VStarMoveBackwards = probMoveBackwards * (stepCost + (discountFactor*previousValue[backwardX][backwardY]))
	VStarMoveLeft = probMoveLeftOfIntended * (stepCost + (discountFactor*previousValue[leftX][leftY]))
	VStarMoveRight = probMoveRightOfIntended * (stepCost + (discountFactor*previousValue[rightX][rightY]))

	VStar = VStarMoveCorrect + VStarMoveBackwards + VStarMoveLeft + VStarMoveRight

	#need to check if it is in the grid or if it is a wall/pit/goal


def calculateMaxVStar(x,y):
	VstarArray = []
	direction = []
	direction[0] = "up"
	direction[1] = "down"
	direction[2] = "left"
	direction[3] = "right"

	for i in range(0, numPossibleMoves):
		VstarArray[i] = calculateVStar(x,y, direction[i])

	maxVStar = numpy.amax(VstarArray)

	return maxVStar




