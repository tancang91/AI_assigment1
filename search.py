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

class Node:
    def __init__(self, data, parent=None):
        self.data = data
        self.parent = parent if parent is not None else None
        self.cost = 0



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


def get_path(node):
    path = []
    temp = node
    while temp is not None:
        path.append(temp.data[1])
        temp = temp.parent

    path.pop()
    l = 0
    r = len(path) - 1
    while (l < r):
        path[l], path[r] = path[r], path[l]
        l += 1; r -= 1

    return path

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
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    from util import Stack

    frontier = Stack()
    visited_list = set()

    start_state = problem.getStartState()
    frontier.push(Node((start_state,None,0), None))

    current_state = None
    node = None
    is_goal = False
    while(not frontier.isEmpty() and not is_goal):
        node = frontier.pop()
        current_state = node.data

        if not current_state[0] in visited_list:
            visited_list.add(current_state[0])

        if problem.isGoalState(current_state[0]):
            is_goal = True
            break

        successors = problem.getSuccessors(current_state[0])
        for successor in successors:
            if not successor[0] in visited_list:
                frontier.push(Node(successor, node))

    path = []
    if is_goal:
        path = get_path(node)

    return path

def breadthFirstSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import Queue

    start_state = problem.getStartState()

    frontier = Queue()
    parent = {}
    current_cost = {}

    frontier.push(start_state)
    parent[start_state] = None
    current_cost[start_state] = 0

    goal = None
    while(not frontier.isEmpty()):
        current_state = frontier.pop()

        if problem.isGoalState(current_state):
            goal = current_state
            break

        successors = problem.getSuccessors(current_state)
        for successor in successors:
            next_state = successor[0]
            action = successor[1]
            cost = successor[2]
            new_cost = current_cost[current_state] + cost
            if not next_state in current_cost:
                current_cost[next_state] = new_cost
                parent[next_state] = (current_state, action)
                frontier.push(next_state)

    path = []
    current = goal
    if goal is not None:
        while current != start_state:
            parent_location = parent[current][0]
            action = parent[current][1]

            current = parent_location
            path.append(action)
        path = list(reversed(path))

    return path

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue

    start_state = problem.getStartState()

    frontier = PriorityQueue()
    parent = {}
    current_cost = {}

    frontier.push(start_state,0)
    parent[start_state] = None
    current_cost[start_state] = 0

    goal = None
    while(not frontier.isEmpty()):
        current_state = frontier.pop()

        if problem.isGoalState(current_state):
            goal = current_state
            break

        successors = problem.getSuccessors(current_state)
        for successor in successors:
            next_state = successor[0]
            action = successor[1]
            cost = successor[2]
            new_cost = current_cost[current_state] + cost
            if not next_state in current_cost or new_cost < current_cost[next_state]:
                current_cost[next_state] = new_cost
                parent[next_state] = (current_state, action)

                # UCS is dijkstra algorithms, with priority actual cost from start node to node
                priority = new_cost
                frontier.push(next_state, priority)

    path = []
    current = goal
    if goal is not None:
        while current != start_state:
            parent_location = parent[current][0]
            action = parent[current][1]

            current = parent_location
            path.append(action)
        path = list(reversed(path))
    return path


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    # from searchAgents import manhattanHeuristic

    start_state = problem.getStartState()

    frontier = PriorityQueue()
    parent = {}
    current_cost = {}

    frontier.push(start_state,0)
    parent[start_state] = None
    current_cost[start_state] = 0

    goal = None
    while(not frontier.isEmpty()):
        current_state = frontier.pop()

        if problem.isGoalState(current_state):
            goal = current_state
            break

        successors = problem.getSuccessors(current_state)
        for successor in successors:
            next_state = successor[0]
            action = successor[1]
            cost = successor[2]

            new_cost = current_cost[current_state] + cost # Actual cost of child node

            if not next_state in current_cost or new_cost  < current_cost[next_state]:
                current_cost[next_state] = new_cost
                parent[next_state] = (current_state, action)

                # A star is dijkstra algorithms extention include heuristic cost
                priority = new_cost + heuristic(next_state, problem)
                frontier.push(next_state, priority)

    path = []
    current = goal
    if goal is not None:
        while current != start_state:
            parent_location = parent[current][0]
            action = parent[current][1]

            current = parent_location
            path.append(action)
        path = list(reversed(path))
    return path

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
