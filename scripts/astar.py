#!/usr/bin/env python

# astar implementation needs to go here

import rospy
from read_config import read_config

def AStarFindPath():
	self.config = read_config()

	self.cellPublisher = rospy.Publisher(

			"/results/path_list",

			AStarPath,

			queue_size = 20

		)


	mapSize = config['map_size']
	rows = mapSize[0]
	cols = mapSize[1]
	start = config['start']
	start_X = start[0]
	start_Y = start[1]
	goal = config['goal']
	goal_X = goal[0]
	goal_Y = goal[1]
	walls = config['walls']
	walls_X = walls[0]
	walls_Y = walls[1]
	pits = config['pits']
	pits_X = pits[0]
	pits_Y = pits[1]

	unicost_grid = [rows][cols] #grid to hold cost values
	heuristic_grid = [rows][cols] #grid to hold heuristic values
	astar_grid = [rows][cols] #grid to hold the astar values


	#calculating uniform cost grid
	for i in range(0,rows):
		for j in range(0,cols):
			valX = abs(i - start_X)
			valY = abs(j - start_Y)
			total = valX + valY
			unicost_grid[i][j] = total

	#calculating heuristic grid
	for r1 in range(0,rows):
		for c1 in range(0,cols):
			heurX = abs(r1 - goal_X)
			heurY = abs(c1 - goal_Y)
			heurTotal = heurX + heurY
			heuristic_grid[r1][c1] = heurTotal

	#calculating astar grid
	for r2 in range(0,rows):
		for c2 in range(0,cols):
			astarVal = unicost_grid[r2][c2] + heuristic_grid[r2][c2]
			astar_grid[r2][c2] = astarVal

	#calculating minimum cost 
	#while current grid is not the goal
	#explore the neighbors of the current grid to find the path 
	#to get to the goal
	currentGrid = start
	currentGrid_x = start_X
	currentGrid_y = start_Y
	closed_list = [] #elements that will eventually be published
	neighbor_list = [] #neighbors of the currentGrid

	while currentGrid != goal:
		#adds starting grid to closed list as first element
		if currentGrid = start:
_			closed_list.append(start)
		else:
			#get neighbors
			if (currentGrid_x - 1 >= 0 & currentGrid_x - 1 <= rows & currentGrid_y >= 0 & currentGrid_y <= cols
				& currentGrid_x - 1 != pits_X & currentGrid_x - 1 != walls_X & currentGrid_y != pits_Y & currentGrid_y != walls_Y):
				neighbor_list.append([currentGrid_x - 1, currentGrid_y])
			else if (currentGrid_x >= 0 & currentGrid_x <= rows & currentGrid_y + 1 >= 0 & currentGrid_y + 1 <= cols
				& currentGrid_x != pits_X & currentGrid_x != walls_X & currentGrid_y + 1 != walls_Y & currentGrid_y + 1 != pits_Y):
				neighbor_list.append([currentGrid_x, currentGrid_y + 1])
			else if (currentGrid_x + 1 >= 0 & currentGrid_x + 1 <= rows & currentGrid_y >= 0 & currentGrid_y <= cols
				& currentGrid_x + 1 != pits_X & currentGrid_x + 1 != walls_X & currentGrid_y != walls_Y & currentGrid_y != pits_Y):
				neighbor_list.append([currentGrid_x + 1, currentGrid_y])
			else if (currentGrid_x >= 0 & currentGrid_x <= rows & currentGrid_y - 1 >= 0 & currentGrid_y - 1 <= cols
				& currentGrid_x != walls_X & currentGrid_x != pits_X & currentGrid_y - 1 != walls_Y & currentGrid_y - 1 != pits_Y):
				neighbor_list.append([currentGrid_x, currentGrid_y - 1])

			minimum = find_min(neighbor_list) #gets back index of which neighbor has minimum value
			closed_list.append(neighbor_list[minimum]) #adds neighbor with min value to closed list

		#currentGrid's lowest cost neighbor becomes the currentGrid
		currentGrid = neighbor_list[minimum]
		


	#finds the minimum value
	def find_min(neighbor_list):
		minval = neighbor_list[0]
		index = 0
		for i in range(0, len(neighborlist)):
			if neighbor_list[i] < minval:
				minval = neighborlist[i]
				index = i
		return index
	#calculate uniform costs for grids
	#calculate heuristic costs for grids
	#sum them and come up with A star costs for grids
	#pick minimal cost from neighbor of start and publish that move
	#from there pick neighbor with minimal cost and publish
	#repeat until end up in the goal