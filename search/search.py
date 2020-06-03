# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    stack = util.Stack()
    visitedNodes = []
    state = problem.getStartState()
    stack.push((state, []))		#Pushing the state and the actions taken to reach it on top of the stack
    
    while not stack.isEmpty():
        currState, actionsToGoal = stack.pop()
        if problem.isGoalState(currState):
            return actionsToGoal
        visitedNodes.append(currState)
        for successor in problem.getSuccessors(currState):	# 0th index has the next state, 1st index has its the direction
                if successor[0] not in visitedNodes:
                    state = successor[0]
                    move = successor[1]
                    stack.push((successor[0], actionsToGoal + [move]))
    
# ________________________________________________________________
class _RecursiveDepthFirstSearch(object):
    '''
        => Output of 'recursive' dfs should match that of 'iterative' dfs you implemented
        above. 
        Key Point: Remember in tutorial you were asked to expand the left-most child 
        first for dfs and bfs for consistency. If you expanded the right-most
        first, dfs/bfs would be correct in principle but may not return the same
        path. 

        => Useful Hint: self.problem.getSuccessors(node) will return children of 
        a node in a certain "sequence", say (A->B->C), If your 'recursive' dfs traversal 
        is different from 'iterative' traversal, try reversing the sequence.  

    '''
    def __init__(self, problem):
        " Do not change this. " 
        # You'll save the actions that recursive dfs found in self.actions. 
        self.actions = [] 
        # Use self.explored to keep track of explored nodes.  
        self.explored = set()
        self.problem = problem

    def RecursiveDepthFirstSearchHelper(self, node):
        self.explored.add(node)
        if self.problem.isGoalState(node):
            return True
        for successor in self.problem.getSuccessors(node)[::-1]:	# 0th index has the next state, 1st index has its the direction
                if successor[0] not in self.explored:
                    self.explored.add(successor[0])
                    if self.RecursiveDepthFirstSearchHelper(successor[0]):
                        self.actions.append(successor[1])	
                        return True
        return False


def RecursiveDepthFirstSearch(problem):
    " You need not change this function. "
    # All your code should be in member function 'RecursiveDepthFirstSearchHelper' of 
    # class '_RecursiveDepthFirstSearch'."

    node = problem.getStartState() 
    rdfs = _RecursiveDepthFirstSearch(problem)
    path_found = rdfs.RecursiveDepthFirstSearchHelper(node)
    return list(reversed(rdfs.actions)) # Actions your recursive calls return are in opposite order.
# ________________________________________________________________


def depthLimitedSearch(problem, limit = 70):	#For big maze limit = 210
    queue = util.Queue()
    startState = problem.getStartState()
    queue.push((startState, [], 0))
    visited = []
    visited.append(startState)
    while not queue.isEmpty():
            state, actions, depth = queue.pop()
            if depth >= limit:
                return []
            for successor in problem.getSuccessors(state):
                    if successor[0] not in visited:
                        if problem.isGoalState(successor[0]):
                            return actions + [successor[1]]
                        else: 
                            queue.push((successor[0], actions + [successor[1]], depth + 1))
                            visited.append(successor[0]) 			



# ________________________________________________________________

class _RecursiveDepthLimitedSearch(object):
    '''
        => Output of 'recursive' dfs should match that of 'iterative' dfs you implemented
        above. 
        Key Point: Remember in tutorial you were asked to expand the left-most child 
        first for dfs and bfs for consistency. If you expanded the right-most
        first, dfs/bfs would be correct in principle but may not return the same
        path. 

        => Useful Hint: self.problem.getSuccessors(node) will return children of 
        a node in a certain "sequence", say (A->B->C), If your 'recursive' dfs traversal 
        is different from 'iterative' traversal, try reversing the sequence.  

    '''
    def __init__(self, problem):
        " Do not change this. " 
        # You'll save the actions that recursive dfs found in self.actions. 
        self.actions = [] 
        # Use self.explored to keep track of explored nodes.  
        self.explored = set()
        self.problem = problem
        self.current_depth = 0
        self.depth_limit = 204  # For medium maze, You should find solution for depth_limit not more than 204.

    def RecursiveDepthLimitedSearchHelper(self, node):
        depth = self.current_depth
        self.explored.add(node)
        if self.problem.isGoalState(node):
                return True
        else:
            for succssor in self.problem.getSuccessors(node)[::-1]:
                    if succssor[0] not in self.explored:
                            self.explored.add(succssor[0])
                            self.current_depth = self.current_depth + 1
                            if self.depth_limit >= self.current_depth and self.RecursiveDepthLimitedSearchHelper(succssor[0]):
                                    self.actions.append(succssor[1])
                                    return True
                            self.current_depth = depth
        return False


def RecursiveDepthLimitedSearch(problem):
    "You need not change this function. All your code in member function RecursiveDepthLimitedSearchHelper"
    node = problem.getStartState() 
    rdfs = _RecursiveDepthLimitedSearch(problem)
    path_found = rdfs.RecursiveDepthLimitedSearchHelper(node)
    return list(reversed(rdfs.actions)) # Actions your recursive calls return are in opposite order.
# ________________________________________________________________


def breadthFirstSearch(problem):
    queue = util.Queue()
    visitedNodes = []
    start = problem.getStartState()
    queue.push((start, []))		#Pushing the state and the actions taken to reach it on top of the stack
    visitedNodes.append(start)
    
    while not queue.isEmpty():
        currState, actionsToGoal = queue.pop()
        if problem.isGoalState(currState):
            return actionsToGoal
        for successor in problem.getSuccessors(currState):	# 0th index has the next state, 1st index has its the direction
                if successor[0] not in visitedNodes:
                        queue.push((successor[0], actionsToGoal + [successor[1]]))
                        visitedNodes.append(successor[0])

def uniformCostSearch(problem):
    startNode = problem.getStartState()
    queue = util.PriorityQueue()
    queue.push((startNode, []), 0)
    visited = []
    visited.append(startNode)

    while not queue.isEmpty():
            (state, actions), cost = queue.pop()
            if problem.isGoalState(state):
                return actions
            for successor in problem.getSuccessors(state):
                    successorState = successor[0]
                    successorActions = [successor[1]]
                    successorCost = successor[2]
                    if successorState not in visited:
                            if successorState not in queue.heap:
                                    queue.push((successorState, actions + successorActions), successorCost + cost)
                                    visited.append(successorState)
                    if queue.item_present_with_higher_priority(successor[0], successorCost + cost):
                            index, count = queue.item_present_with_higher_priority(successor[0], successorCost + cost)
                            queue.Update_priority((successorState, actions + successorActions), successorCost + cost, index, count)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    startNode = problem.getStartState()
    queue = util.PriorityQueue()
    cost = heuristic(startNode, problem)
    queue.push((startNode, []), cost)
    visited = []
    visited.append(startNode)

    while not queue.isEmpty():
            (state, actions), cost = queue.pop()
            cost = cost - heuristic(state, problem)
            if problem.isGoalState(state):
                return actions
            for successor in problem.getSuccessors(state):
                    successorState = successor[0]
                    successorActions = [successor[1]]
                    successorCost = successor[2]
                    if successorState not in visited:
                            if successorState not in queue.heap:
                                    queue.push((successorState, actions + successorActions), successorCost + cost + heuristic(successorState, problem))
                                    visited.append(successorState)
                    if queue.item_present_with_higher_priority(successor[0], successorCost + cost + heuristic(successorState, problem)):
                            index, count = queue.item_present_with_higher_priority(successor[0], successorCost + cost + heuristic(successorState, problem))
                            queue.Update_priority((successorState, actions + successorActions), successorCost + cost + heuristic(successorState, problem), index, count)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
rdfs = RecursiveDepthFirstSearch
dls = depthLimitedSearch
rdls = RecursiveDepthLimitedSearch
astar = aStarSearch
ucs = uniformCostSearch
