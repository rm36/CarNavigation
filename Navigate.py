#Implementation of A* Algorithm

import math
from platform import node

from queue import PriorityQueue

class Node:

    

    def __init__(self, x, y, g, h):
        self.x = x
        self.y = y
        # g is cost of path from one node to another
        self.g = g
        # h is the heuristic a guess of what it would possibly cost to get to endpoint
        self.h = h
        self.f = g + h

    def compareTo(self, that):
        return self.f - that.f
        
    

    def aStarSearch(grid):
        proximity_coords = [(0,-1), (1,-1), (1,0), (1,1), (0,1), (-1, 1), (-1, 0), (-1, -1)]
        m = len(grid)
        n = len(grid[0])
        endXCoord = m - 1
        endYCoord = n - 1

        if grid[endXCoord][endYCoord] == 1:
            return -1 #Exit Grid and Search early

        #PriorityQueue for nodes 
        nodePriorityQueue = PriorityQueue()

        #create closed list of nodes that are visited
        closed = []

        #starting node gets added to queue
        nodePriorityQueue.put(Node(0, 0, 1, max(m, n)))

        while not nodePriorityQueue.empty():
            # Retrieve Node with smallest f value 
            someNode = nodePriorityQueue.get()
            x, y = someNode.x, someNode.y
            if x < 0 or x >= m or y < 0 or y >= n or grid[x][y] == 1:
                continue

            #if the node has reached the goal, stop searching
            if x == endXCoord and y == endYCoord:
                return someNode.g

            #if a node at the same position is found within the closed list with a lower or equal g value, skip adding to closed
            if closed[x * m + y] <= someNode.g:
                continue

            #push node to closed list
            closed[x * m + y] = someNode.g

            #generate closest nodes
            for x2,y2 in proximity_coords:
                # for each successor take prev node and add distance between successor and node
                # estimate for successor.h distance from sucessor to goal
                g = someNode.g + 1

                # Use Diag Distance as heuristic
                h = max(abs(endXCoord - x), abs(endYCoord - y))

                # Add successor to priority queue
                nodePriorityQueue.put(Node(x + x2, y+y2, g, h))
        return -1



