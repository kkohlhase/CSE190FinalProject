#!/usr/bin/env python

# astar implementation needs to go here

import rospy
from read_config import read_config

def AStarFindPath():
	self.config = read_config()

	self.cellPublisher = rospy.Publisher(

			"/results/path_list",

			AStarPath,

			queue_size = 10

		)


	mapSize = self.config['map_size']

	rows = mapSize[0]
	cols = mapSize[1]


	#calculate uniform costs for grids
	#calculate heuristic costs for grids
	#sum them and come up with A star costs for grids
	#pick minimal cost from neighbor of start and publish that move
	#from there pick neighbor with minimal cost and publish
	#repeat until end up in the goal