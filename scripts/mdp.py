#!/usr/bin/env python

# mdp implementation needs to go here

import rospy
from read_config import read_config

config = read_config()

stepCost = config['reward_for_each_step']

probMoveCorrect = config['prob_move_forward']
probMoveOpposite = config['prob_move_backward']
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

def MDPFindPolicies():

	#calculate the first value iteration

	#publish

	#update original grid with new values

	#repeat until convergence or reached maximum number of iterations

def calculateVStar():

def calculateMaxVStar():