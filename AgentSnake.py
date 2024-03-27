#Libraries
import heapq
import math
import queue
from State import Vector
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Unified Cost Search
#Agent 
class Agent(object):
    def SearchSolution(self, state):
        #passing it 
        return []
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class UCSAgent(Agent):
    def _init_(self):
        self.last_move = None 
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def SearchSolution(self, state):
        FoodX = state.FoodPosition.X
        FoodY = state.FoodPosition.Y
        HeadX = state.snake.HeadPosition.X
        HeadY = state.snake.HeadPosition.Y
        plan = []
#unified
        #cost
             #search
        startingNode = (HeadX, HeadY)
        goalNode = (FoodX, FoodY)
        tempQueue = queue.PriorityQueue()
        tempQueue.put((startingNode, 0))  
        startNode = {}
        cost = {}
        startNode[startingNode] = None
        cost[startingNode] = 0

        while not tempQueue.empty():
            #fetch the least coast node
            currentNodeLocation, currentCost = tempQueue.get()  
#senario to find foooddd yummyyy
            if currentNodeLocation == goalNode:
                break
            
#neigbour nodes
            for nexttNodeToBe in self.fetchNeighbour(state.maze, currentNodeLocation):
                #cost of each step = old cost + 1
                newCost = currentCost + 1 
#what if snake is hit by it self
                newHeadPos = Vector(nexttNodeToBe[0], nexttNodeToBe[1])
                if newHeadPos not in state.snake.Body:
#if it dont only then
                    if nexttNodeToBe not in cost or newCost < cost[nexttNodeToBe]:
                        cost[nexttNodeToBe] = newCost
                        priority = newCost
                        tempQueue.put((nexttNodeToBe, newCost))
                        startNode[nexttNodeToBe] = currentNodeLocation

#path
        currentNodeLocation = goalNode
        while currentNodeLocation != startingNode:
            nextMove = self.next_direction(startNode[currentNodeLocation], currentNodeLocation)
            plan.append(nextMove)
            currentNodeLocation = startNode[currentNodeLocation]

#finding the plan
        plan.reverse()
        return plan
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def next_direction(self, currentNodeLocation, nexttNodeToBe):
    #wrong direction check
        if currentNodeLocation is None:
            return -1
    #x axis
        dx = nexttNodeToBe[0] - currentNodeLocation[0]
        dy = nexttNodeToBe[1] - currentNodeLocation[1]

        if dx == 1:
            return 3  # Right
        elif dx == -1:
            return 9  # Left
        elif dy == 1:
            return 6  # Down
        elif dy == -1:
            return 0  # Up
        else:
            return -1  # No valid direction
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++     
    def fetchNeighbour(self, maze, node):
#all possible directions
        neighbors = []
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  

        for direction in directions:
            new_x = node[0] + direction[0]
            new_y = node[1] + direction[1]
#isValid OR not
            if 0 <= new_x < maze.WIDTH and 0 <= new_y < maze.HEIGHT and maze.MAP[new_y][new_x] != -1:
                neighbors.append((new_x, new_y))
                

        return neighbors


    def showAgent():
        print("A Snake Solver By Anam n Omer")

#A*

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
class Agent(object):
    def SearchSolution(self, state):
        return []
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
class AstarSearch(Agent):
    def __init__(self):
        self.last_move = None  # Initialize last_move attribute
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    def SearchSolution(self, state):
        FoodX = state.FoodPosition.X
        FoodY = state.FoodPosition.Y

        HeadX = state.snake.HeadPosition.X
        HeadY = state.snake.HeadPosition.Y

        plan = []
        #if snake has body or not or like its size exits
        if state.snake.Size > 1:  
            newHeadPos = Vector(HeadX + state.snake.HeadDirection.X, HeadY + state.snake.HeadDirection.Y)
            if newHeadPos in state.snake.Body:
                return []

# A* implementation
        startingNode = (HeadX, HeadY)
        goalNode = (FoodX, FoodY)

        frontier = queue.PriorityQueue()
        frontier.put((0, startingNode)) 

        startedFrom = {}
        tillPositionCost = {}
        startedFrom[startingNode] = None
        tillPositionCost[startingNode] = 0

        while not frontier.empty():
            _, currentNodeLocation = frontier.get()  

            if currentNodeLocation == goalNode:
                break
#generte neighbour
            for nexttNodeToBe in self.fetchNeighbour(state.maze, currentNodeLocation):
                #cost
                new_cost = tillPositionCost[currentNodeLocation] + 1  


                newHeadPos = Vector(nexttNodeToBe[0], nexttNodeToBe[1])
                if newHeadPos not in state.snake.Body:

                    if nexttNodeToBe not in tillPositionCost or new_cost < tillPositionCost[nexttNodeToBe]:
                        tillPositionCost[nexttNodeToBe] = new_cost
                        priority = new_cost + self.heuristic(nexttNodeToBe, goalNode)
                        frontier.put((priority, nexttNodeToBe))
                        startedFrom[nexttNodeToBe] = currentNodeLocation

#new path
        currentNodeLocation = goalNode
        while currentNodeLocation != startingNode:
            nextMove = self.targetToReach(startedFrom[currentNodeLocation], currentNodeLocation)
            plan.append(nextMove)
            currentNodeLocation = startedFrom[currentNodeLocation]

#get plan
        plan.reverse()
        return plan
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    def heuristic(self, currentNodeLocation, goalNode):
        return abs(currentNodeLocation[0] - goalNode[0]) + abs(currentNodeLocation[1] - goalNode[1])
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    def targetToReach(self, currentNodeLocation, nexttNodeToBe):
#invalid move
        if currentNodeLocation is None:
            return -1  
#dy and dx
        dx = nexttNodeToBe[0] - currentNodeLocation[0]
        dy = nexttNodeToBe[1] - currentNodeLocation[1]

        if dx == 1:
            return 3  # Right
        elif dx == -1:
            return 9  # Left
        elif dy == 1:
            return 6  # Down
        elif dy == -1:
            return 0  # Up
        else:
            return -1  # No valid direction
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    def fetchNeighbour(self, maze, currentNodeLocation):
        neighbors = []
        #possible moves
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  
        
        for direction in directions:
            new_x = currentNodeLocation[0] + direction[0]
            new_y = currentNodeLocation[1] + direction[1]
            #debug line 
            print("Checking node:", (new_x, new_y))  
            
            if 0 <= new_x < maze.WIDTH and 0 <= new_y < maze.HEIGHT and maze.MAP[new_y][new_x] != -1:
                neighbors.append((new_x, new_y))
        
        return neighbors


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    def fetchDirectOpp(self, direction):
        if direction == 0:
            return 6  # Opposite of Up is Down
        elif direction == 6:
            return 0  # Opposite of Down is Up
        elif direction == 3:
            return 9  # Opposite of Right is Left
        elif direction == 9:
            return 3  # Opposite of Left is Right
        else:
            return None  # No opposite direction
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    def showAgent():
        print("A Snake Solver By Anam n Omer")


#BFS
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
class Agent(object):
    def SearchSolution(self, state):
        return []
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
class BFSAgent(Agent):
    def _init_(self):
        self.last_move = None  
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    def SearchSolution(self, state):
        FoodX = state.FoodPosition.X
        FoodY = state.FoodPosition.Y

        HeadX = state.snake.HeadPosition.X
        HeadY = state.snake.HeadPosition.Y

        plan = []
        if state.snake.Size > 1:  
            next_head_position = Vector(HeadX + state.snake.HeadDirection.X, HeadY + state.snake.HeadDirection.Y)
            if next_head_position in state.snake.Body:
                return []

        # BFS implementation
        start_node = (HeadX, HeadY)
        goal_node = (FoodX, FoodY)

        frontier = queue.Queue()
        frontier.put(start_node)  

        came_from = {}
        came_from[start_node] = None

        while not frontier.empty():
            current_node = frontier.get() 

            if current_node == goal_node:
                break
            for next_node in self.get_neighbors(state.maze, current_node):
                if next_node not in came_from:
                    frontier.put(next_node)
                    came_from[next_node] = current_node

#NEW PATH
        current_node = goal_node
        while current_node != start_node:
            next_move = self.direction_to_reach(came_from[current_node], current_node)
            plan.append(next_move)
            current_node = came_from[current_node]

#GET PLAN
        plan.reverse()
        return plan
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    def direction_to_reach(self, current_node, next_node):

        if current_node is None:
            return -1  

        dx = next_node[0] - current_node[0]
        dy = next_node[1] - current_node[1]

        if dx == 1:
            return 3  # Right
        elif dx == -1:
            return 9  # Left
        elif dy == 1:
            return 6  # Down
        elif dy == -1:
            return 0  # Up
        else:
            return -1  # No valid direction
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    def get_neighbors(self, maze, node):
        neighbors = []
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)] 

        for direction in directions:
            new_x = node[0] + direction[0]
            new_y = node[1] + direction[1]

            if 0 <= new_x < maze.WIDTH and 0 <= new_y < maze.HEIGHT and maze.MAP[new_y][new_x] != -1:
                neighbors.append((new_x, new_y))

        return neighbors

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    def get_opposite_direction(self, direction):
        if direction == 0:
            return 6  # Opposite of Up is Down
        elif direction == 6:
            return 0  # Opposite of Down is Up
        elif direction == 3:
            return 9  # Opposite of Right is Left
        elif direction == 9:
            return 3  # Opposite of Left is Right
        else:
            return None  # No opposite direction
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    def showAgent():
        print("A Snake Solver By Anam")



#DFS
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
class Agent(object):
    def SearchSolution(self, state):
        return []
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
class DFSAgent(Agent):
    def __init__(self):
        self.last_move = None  
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    def SearchSolution(self, state):
        FoodX = state.FoodPosition.X
        FoodY = state.FoodPosition.Y

        HeadX = state.snake.HeadPosition.X
        HeadY = state.snake.HeadPosition.Y

        plan = []

 #snake check
        if state.snake.Size > 1:  # Only check collision if the snake has a body
            newHeadPos = Vector(HeadX + state.snake.HeadDirection.X, HeadY + state.snake.HeadDirection.Y)
            if newHeadPos in state.snake.Body:
                return []

        # DFS
        startingNode = (HeadX, HeadY)
        goalNode = (FoodX, FoodY)
#STACK
        frontier = [startingNode]  

        startedFrom = {}
        startedFrom[startingNode] = None
#FRONTIER NODE
        while frontier:
            currentNodeLocation = frontier.pop()  

            if currentNodeLocation == goalNode:
                break

# Generate neighboring nodes
            for nexttNodeToBe in self.fetchNeighbour(state.maze, currentNodeLocation):
                if nexttNodeToBe not in startedFrom:
                    frontier.append(nexttNodeToBe)
                    startedFrom[nexttNodeToBe] = currentNodeLocation

#NEW PATH
        currentNodeLocation = goalNode
        while currentNodeLocation != startingNode:
            nextMove = self.targetToReach(startedFrom[currentNodeLocation], currentNodeLocation)
            plan.append(nextMove)
            currentNodeLocation = startedFrom[currentNodeLocation]

#GET ORDER
        plan.reverse()
        return plan
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    def targetToReach(self, currentNodeLocation, nexttNodeToBe):
        if currentNodeLocation is None:
            return -1  # Return an invalid direction if currentNodeLocation is None

        dx = nexttNodeToBe[0] - currentNodeLocation[0]
        dy = nexttNodeToBe[1] - currentNodeLocation[1]

        if dx == 1:
            return 3  # Right
        elif dx == -1:
            return 9  # Left
        elif dy == 1:
            return 6  # Down
        elif dy == -1:
            return 0  # Up
        else:
            return -1  # No valid direction
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    def fetchNeighbour(self, maze, node):
        neighbors = []
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # Up, Down, Left, Right

        for direction in directions:
            new_x = node[0] + direction[0]
            new_y = node[1] + direction[1]

            if 0 <= new_x < maze.WIDTH and 0 <= new_y < maze.HEIGHT and maze.MAP[new_y][new_x] != -1:
                neighbors.append((new_x, new_y))

        return neighbors
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    def fetchDirectOpp(self, direction):

        if direction == 0:
            return 6  # Opposite of Up is Down
        elif direction == 6:
            return 0  # Opposite of Down is Up
        elif direction == 3:
            return 9  # Opposite of Right is Left
        elif direction == 9:
            return 3  # Opposite of Left is Right
        else:
            return None  # No opposite direction
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    def showAgent():
        print("A Snake Solver By Omer")
