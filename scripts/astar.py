#!/usr/bin/env python

from read_config import read_config
from cse_190_assi_3.msg import *
from copy import deepcopy
import rospy

class Astar():

    def __init__(self):
        self.config = read_config()
        self.pathPublisher = rospy.Publisher(
            "/results/path_list",

            AStarPath,

            queue_size = 20
        )
        
        self.mapSize = self.config['map_size']
        self.moveList = self.config['move_list']
        self.numRows = self.mapSize[0]
        self.numCols = self.mapSize[1]
        self.start = self.config['start']
        self.goal = self.config['goal']
        self.walls = self.config['walls']
        self.pits = self.config['pits']
        self.originalGrid =[[0 for x in range(self.numCols)] for y in range(self.numRows)]
        self.uniformCost = 0
        
        for m,n in self.walls:

			self.originalGrid[m][n] = -1
            
        for o,p in self.pits:
            self.originalGrid[o][p] = -10
            
        self.originalGrid[self.goal[0]][self.goal[1]] = 10
        
        self.originalGrid[self.start[0]][self.start[1]] = 1
        
        currentGrid = self.start
        self.publishPath(currentGrid)

        #print 'Before while loop, published ', currentGrid
        #self.counter = 0
        while (currentGrid != self.goal):
            #print'Entering while loop'
            self.uniformCost = self.uniformCost + 1
            possibleMoves = []
            for neighbor in self.getNeighbors(currentGrid):
                totalCost = self.heuristicDistance(neighbor) + self.uniformCost
                possibleMoves.append([totalCost, neighbor])
                #print 'neighbors are ', neighbor
            bestMove = min(possibleMoves)
            newGrid = bestMove[1]
            currentGrid = newGrid
            self.publishPath(currentGrid)
            #self.counter = self.counter + 1
            #if self.counter > 20:
                #break
            #print 'End of while loop, published ', currentGrid, bestMove[0]
        
        
    def heuristicDistance (self, currentGrid):
        heuristicCost = abs(currentGrid[0] - self.goal[0]) + abs(currentGrid[1] - self.goal[1])
        return heuristicCost
        
    def getNeighbors (self, currentGrid):
        neighbors = []
        x, y = currentGrid
        possibleNeighbors = []
        for move in self.moveList:
            newX = x+move[0]
            newY = y+move[1]
            possibleNeighbors.append([newX, newY])

        for neighbor in possibleNeighbors:
            row, col = neighbor
            if row < self.numRows and row >= 0  and col < self.numCols and col >= 0 and self.originalGrid[row][col] >= 0:
                neighbors.append(neighbor)
        #print 'neighbors are ', neighbors
        return neighbors
        
    def publishPath(self, currentGrid):
        #print 'Should be publishing'
        path = AStarPath()
        path.data = currentGrid
        #print path.data
        rospy.sleep(1)
        self.pathPublisher.publish(path)
        
        
