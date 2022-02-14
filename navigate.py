# Implementation of A* Algorithm
from queue import PriorityQueue
import copy

DEBUGGING = False

class Node:
    def __init__(self, x, y, parentNode = None):
        self.x = x
        self.y = y

        # Aggregated path for solution
        self.path = []
        if parentNode is not None:
            self.path = copy.deepcopy(parentNode.path)
        self.path.append((x,y))

    def __lt__(self, other):
        return self.x < other.x


def is_obstacle(cell):
    return cell == 1

def is_coord_in_grid(coord, grid):
    (x, y) = coord
    (m, n) = grid.shape
    return x >= 0 and x < m and y >= 0 and y < n

def a_star_search(grid, startPosition, endPosition):
    proximity_coords = [(0,-1), (1,-1), (1,0), (1,1), (0,1), (-1, 1), (-1, 0), (-1, -1)]
    (m, n) = grid.shape
    (endXCoord, endYCoord) = endPosition

    if grid[endXCoord, endYCoord] == 1:
        return -1 # Exit Grid and Search early

    # PriorityQueue based on distance to target
    nodePriorityQueue = PriorityQueue()

    firstNodeDiagDistance = 0 # Arbitrary because it gets popped right away
    firstNode = Node(startPosition[0], startPosition[1])

    alreadyAdded = {startPosition}
    nodePriorityQueue.put((firstNodeDiagDistance, firstNode))

    while not nodePriorityQueue.empty():
        # Retrieve Node with smallest distance to target
        (nodeDiagDistance, currentNode) = nodePriorityQueue.get()
        
        if DEBUGGING:
            print('next item:', (nodeDiagDistance, (currentNode.x, currentNode.y)))
        x, y = currentNode.x, currentNode.y

        # Skip cells on boundary or obstacles
        if not is_coord_in_grid((x,y), grid) or is_obstacle(grid[x, y]):
            continue

        # If the node has reached the goal, stop searching
        if x == endXCoord and y == endYCoord:
            return currentNode.path

        # Iterate all neighbor cells
        for x2, y2 in proximity_coords:
            (succX, succY) = (x + x2, y + y2)

            if (succX, succY) in alreadyAdded:
                continue

            succesorDiagDistance = max(abs(endXCoord - succX), abs(endYCoord - succY))
            succesorNode = Node(succX, succY, parentNode = currentNode)
            
            if DEBUGGING:
                print('adding:', (succesorDiagDistance, (succesorNode.x, succesorNode.y)))

            alreadyAdded.add((succX, succY))
            nodePriorityQueue.put((succesorDiagDistance, succesorNode))
    return None
