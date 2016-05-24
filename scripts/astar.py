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
	start = self.config['start']
	start_X = start[0]
	start_Y = start[1]
	goal = self.config['goal']
	goal_X = goal[0]
	goal_Y = goal[1]
	walls = self.config['walls']
	pits = self.config['pits']

	unicost_grid = [rows][cols] #grid to hold cost values
	heuristic_grid = [rows][cols] #grid to hold heuristic values
	astar_grid = [rows][cols] #grid to hold the astar values


	#calculating uniform cost grid
	for i in rows:
		for j in cols:
			valX = abs(i - start_X)
			valY = abs(j - start_Y)
			total = valX + valY
			unicost_grid[i][j] = total

	#calculating heuristic grid
	for r1 in rows:
		for c1 in cols:
			heurX = abs(r1 - goal_X)
			heurY = abs(c1 - goal_Y)
			heurTotal = heurX + heurY
			heuristic_grid[r1][c1] = heurTotal

	#calculating astar grid
	for r2 in rows:
		for c2 in cols:
			astarVal = unicost_grid[r2][c2] + heuristic_grid[r2][c2]
			astar_grid[r2][c2] = astarVal

	#calculating minimum cost 
	#while current grid is not the goal
	#explore the neighbors of the current grid to find the path 
	#to get to the goal
	currentGrid = start
	currentGrid_x = start_X
	currentGrid_y = start_Y

	while currentGrid != goal
		#get neighbors
		


	#calculate uniform costs for grids
	#calculate heuristic costs for grids
	#sum them and come up with A star costs for grids
	#pick minimal cost from neighbor of start and publish that move
	#from there pick neighbor with minimal cost and publish
	#repeat until end up in the goal