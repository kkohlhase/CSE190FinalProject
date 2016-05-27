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
	walls_X = walls[0]
	walls_Y = walls[1]
	pits = self.config['pits']
	pits_X = pits[0]
	pits_Y = pits[1]

	unicost_grid = [rows][cols] #grid to hold cost values
	heuristic_grid = [rows][cols] #grid to hold heuristic values
	astar_grid = [rows][cols] #grid to hold the astar values

	counter = 0
	uni_current = start
	uni_currentX = start_X
	uni_currentY = start_Y
	uni_neighborlist = [] # list to hold the neighbors and get the one with the minimum value 
	visitedGrids = [] #grids that have already been assigned minimum count values

	while uni_current != goal & len(uni_neighborlist) != 0:
		#if current grid is the start, set counter to 0
		if uni_current == start:
			counter = 0
		else:
			# check if valid neighbors and haven't been visited already then assign a counter value
			if (uni_currentX - 1 >= 0 & uni_currentX - 1 <= rows & uni_currentY >= 0 & uni_currentY <= cols
				& uni_currentX - 1 != pits_X & uni_currentX - 1 != walls_X & uni_currentY != pits_Y 
				& uni_currentY != walls_Y & !check_visited(uni_currentX, uni_currentY)):
				counter = counter + 1
				uni_neighborlist.append([uni_currentX - 1, uni_currentY, counter])
				unicost_grid[uni_currentX - 1][uni_currentY] = counter
			
			else if (uni_currentX >= 0 & uni_currentX <= rows & uni_currentY + 1 >= 0 & uni_currentY + 1 <= cols
				& uni_currentX != pits_X & uni_currentX != walls_X & uni_currentY + 1 != walls_Y 
				& uni_currentY + 1 != pits_Y & !check_visited(uni_currentX, uni_currentY)):
				counter = counter + 1
				uni_neighborlist.append([uni_currentX, uni_currentY + 1, counter])
				unicost_grid[uni_currentX][uni_currentY + 1] = counter

			else if (uni_currentX + 1 >= 0 & uni_currentX + 1 <= rows & uni_currentY >= 0 & uni_currentY <= cols
				& uni_currentX + 1 != pits_X & uni_currentX + 1 != walls_X & uni_currentY != walls_Y 
				& uni_currentY != pits_Y & !check_visited(uni_currentX, uni_currentY)):
				counter = counter + 1
				uni_neighborlist.append([uni_currentX + 1, uni_currentY, counter])
				unicost_grid[uni_currentX + 1][uni_currentY] = counter

			else if (uni_currentX >= 0 & uni_currentX <= rows & uni_currentY - 1 >= 0 & uni_currentY - 1 <= cols
				& uni_currentX != walls_X & uni_currentX != pits_X & uni_currentY - 1 != walls_Y 
				& uni_currentY - 1 != pits_Y & !check_visited(uni_currentX, uni_currentY)):
				counter = counter + 1
				uni_neighborlist.append([uni_currentX, uni_currentY - 1, counter])
				unicost_grid[uni_currentX][uni_currentY - 1] = counter

			min_val_index = find_min(uni_neighborlist)

			visitedGrids.append(uni_neighborlist[min_val_index][1][2]) #adds neighbor with min value to closed list

		#currentGrid's lowest cost neighbor becomes the currentGrid
		uni_current = uni_neighborlist[min_val_index][1][2]

	#check to see if neighbor has already been visited and assigned a count
	def check_visited (xgrid, ygrid):
		for i in range(0, len(visitedGrids)):
			#if neighbor has been visited, return true
			if (visitedGrids[i][1][2] != [xgrid][ygrid]):
				return true
			#if neighbor hasn't been visited, return false
			else:
				return false

	#calculating uniform cost grid
	#for i in range(0,rows):
	#	for j in range(0,cols):
	#		valX = abs(i - start_X)
	#		valY = abs(j - start_Y)
	#		total = valX + valY
	#		unicost_grid[i][j] = total

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
		if currentGrid == start:
_			closed_list.append(start)
		else:
			#get neighbors
			if (currentGrid_x - 1 >= 0 & currentGrid_x - 1 <= rows & currentGrid_y >= 0 & currentGrid_y <= cols
				& currentGrid_x - 1 != pits_X & currentGrid_x - 1 != walls_X & currentGrid_y != pits_Y & currentGrid_y != walls_Y):
				neighbor_list.append([currentGrid_x - 1, currentGrid_y, astar[[currentGrid_x -1], currentGrid_y]])
			else if (currentGrid_x >= 0 & currentGrid_x <= rows & currentGrid_y + 1 >= 0 & currentGrid_y + 1 <= cols
				& currentGrid_x != pits_X & currentGrid_x != walls_X & currentGrid_y + 1 != walls_Y & currentGrid_y + 1 != pits_Y):
				neighbor_list.append([currentGrid_x, currentGrid_y + 1, astar_grid[[currentGrid_x][currentGrid_y + 1]]])
			else if (currentGrid_x + 1 >= 0 & currentGrid_x + 1 <= rows & currentGrid_y >= 0 & currentGrid_y <= cols
				& currentGrid_x + 1 != pits_X & currentGrid_x + 1 != walls_X & currentGrid_y != walls_Y & currentGrid_y != pits_Y):
				neighbor_list.append([currentGrid_x + 1, currentGrid_y, astar_grid[[currentGrid_x + 1][currentGrid_y]]])
			else if (currentGrid_x >= 0 & currentGrid_x <= rows & currentGrid_y - 1 >= 0 & currentGrid_y - 1 <= cols
				& currentGrid_x != walls_X & currentGrid_x != pits_X & currentGrid_y - 1 != walls_Y & currentGrid_y - 1 != pits_Y):
				neighbor_list.append([currentGrid_x, currentGrid_y - 1, astar_grid[[currentGrid_x][currentGrid_y - 1]]])

			minimum = find_min(neighbor_list) #gets back index of which neighbor has minimum value
			#check syntax to get the first two values of the element ********
			closed_list.append(neighbor_list[minimum][1][2]) #adds neighbor with min value to closed list

		#currentGrid's lowest cost neighbor becomes the currentGrid
		currentGrid = neighbor_list[minimum][1][2]
		neighbor_list = []
		
	#publish each grid in the shortest path
	for i in range(0, len(closed_list)):
		cellPublisher.publish(closed_list[i][1][2])

	#finds the minimum value
	def find_min(neighbor_list):
		minval = astar_grid[start]
		index = 0
		for i in range(0, len(neighborlist)):
			if neighbor_list[i][3] < minval:
				minval = neighborlist[i][3]
				index = i
		return index
	#calculate uniform costs for grids
	#calculate heuristic costs for grids
	#sum them and come up with A star costs for grids
	#pick minimal cost from neighbor of start and publish that move
	#from there pick neighbor with minimal cost and publish
	#repeat until end up in the goal