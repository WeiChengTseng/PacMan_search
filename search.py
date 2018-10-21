# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
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
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  init_state = problem.getStartState()
  print('initial state:', init_state)
  print('initail state next:', problem.getSuccessors(init_state))
  print('reach goal:', problem.isGoalState(init_state))

  print(problem.getSuccessors(problem.getStartState()))
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first [p 85].
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm [Fig. 3.7].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  "*** YOUR CODE HERE ***"
  from game import Directions
  from util import Stack

  trajectory = []
  init_state = problem.getStartState()
  state = init_state
  stack = Stack()
  dir_dict = {'South': Directions.SOUTH, 'West': Directions.WEST,
              'North': Directions.NORTH, 'East': Directions.EAST}

  stack.push(state)
  head = {}
  tree = head
  seen = []

  while not stack.isEmpty():
    state = stack.pop()
    
    if problem.isGoalState(state):
      while state != init_state:
        trajectory.append(tree[state]['action'])
        state = tree[state]['prev']
      trajectory = list(reversed(trajectory))
      return trajectory

    for next_state in problem.getSuccessors(state):
      if next_state[0] not in seen:
        stack.push(next_state[0])
        tree[next_state[0]] = {'prev': state, 'action': next_state[1]}
    seen.append(state)


def breadthFirstSearch(problem):
  "Search the shallowest nodes in the search tree first. [p 81]"
  "*** YOUR CODE HERE ***"
  from game import Directions
  from util import Queue

  trajectory = []
  init_state = problem.getStartState()
  state = init_state
  queue = Queue()
  dir_dict = {'South': Directions.SOUTH, 'West': Directions.WEST,
              'North': Directions.NORTH, 'East': Directions.EAST}

  queue.push(state)
  head = {}
  tree = head
  seen = []

  while not queue.isEmpty():
    state = queue.pop()
    
    if problem.isGoalState(state):
      while state != init_state:
        trajectory.append(tree[state]['action'])
        state = tree[state]['prev']
      trajectory = list(reversed(trajectory))
      return trajectory

    for next_state in problem.getSuccessors(state):
      if next_state[0] not in seen:
        queue.push(next_state[0])
        tree[next_state[0]] = {'prev': state, 'action': next_state[1]}
    seen.append(state)

      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"

  from game import Directions
  from util import PriorityQueue
  from util import Counter

  trajectory = []
  init_state = problem.getStartState()
  state = init_state
  queue = PriorityQueue()
  cost_counter = Counter()
  dir_dict = {'South': Directions.SOUTH, 'West': Directions.WEST,
              'North': Directions.NORTH, 'East': Directions.EAST}
  init_cost = 0

  queue.push(state, init_cost)
  cost_counter[init_state] = init_cost
  head = {}
  tree = head
  seen = []

  while not queue.isEmpty():
    state = queue.pop()
    
    if problem.isGoalState(state):
      while state != init_state:
        trajectory.append(tree[state]['action'])
        state = tree[state]['prev']
      trajectory = list(reversed(trajectory))
      return trajectory

    for next_state in problem.getSuccessors(state):
      if next_state[0] not in seen:
        cost_counter[next_state[0]] = cost_counter[state[0]] + next_state[2]
        queue.push(next_state[0], cost_counter[next_state[0]])
        tree[next_state[0]] = {'prev': state, 'action': next_state[1]}
    seen.append(state)

  print('The goal is not reachable!')
  # util.raiseNotDefined()

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"


  from game import Directions
  from util import PriorityQueue
  from util import Counter

  trajectory = []
  init_state = problem.getStartState()
  state = init_state
  queue = PriorityQueue()
  dir_dict = {'South': Directions.SOUTH, 'West': Directions.WEST,
              'North': Directions.NORTH, 'East': Directions.EAST}
  
  init_cost = 0
  real_cost = Counter()
  cost = Counter()


  cost[init_state] = init_cost + nullHeuristic(init_cost)
  queue.push(state, cost[init_state])
  real_cost[init_cost] = init_cost
  tree, seen = {}, []

  while not queue.isEmpty():
    state = queue.pop()
    
    if problem.isGoalState(state):
      while state != init_state:
        trajectory.append(tree[state]['action'])
        state = tree[state]['prev']
      trajectory = list(reversed(trajectory))
      return trajectory

    for next_state in problem.getSuccessors(state):
      if next_state[0] not in seen:
        real_cost[next_state[0]] = real_cost[state[0]] + next_state[2]
        cost[next_state[0]] = real_cost[next_state[0]] + heuristic(next_state[0])
        queue.push(next_state[0], cost[next_state[0]])
        tree[next_state[0]] = {'prev': state, 'action': next_state[1]}
    seen.append(state)

  print('The goal is not reachable!')
  util.raiseNotDefined()
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch