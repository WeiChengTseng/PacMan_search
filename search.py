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
  # define the node as (state, trajectory)
  
  from util import Stack
  stack, seen = Stack(), set()
  stack.push((problem.getStartState(), []))

  while not stack.isEmpty():
    state, trajectory = stack.pop()
    if state in seen:
      continue
    seen.add(state)
    if problem.isGoalState(state):
      return trajectory
    else:
      for successor, action, stepCost in problem.getSuccessors(state):
        if successor not in seen:
          stack.push((successor, trajectory + [action]))

def breadthFirstSearch(problem):
  """
  Search the shallowest nodes in the search tree first.
  """
  # define the node as (state, trajectory)

  from util import Queue
  queue, seen = Queue(), set()
  queue.push((problem.getStartState(), []))

  while not queue.isEmpty():
    state, trajectory = queue.pop()
    if state in seen:
      continue
    seen.add(state)
    if problem.isGoalState(state):
      return trajectory
    else:
      for successor, action, stepCost in problem.getSuccessors(state):
        if successor not in seen:
          queue.push((successor, trajectory + [action]))

def uniformCostSearch(problem):
  # define the node as (state, trajectory)
  from util import PriorityQueue
  from util import Counter
  queue, seen, cost_counter = PriorityQueue(), set(), Counter()
  queue.push((problem.getStartState(), []), 0)
  cost_counter[problem.getStartState()] = 0

  while not queue.isEmpty():
    state, trajectory = queue.pop()
    if state in seen:
      continue
    seen.add(state)
    if problem.isGoalState(state):
      return trajectory
    else:
      for successor, action, cost in problem.getSuccessors(state):
        if successor not in seen:
          cost_counter[successor] = cost + cost_counter[state]
          queue.push((successor, trajectory + [action]), cost_counter[successor])

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"

  # define the node as (state, trajectory)
  from util import PriorityQueue
  from util import Counter
  queue, seen, f, g = PriorityQueue(), set(), Counter(), Counter()
  init_state = problem.getStartState()
  queue.push((init_state, []), 0)
  g[init_state] = 0
  f[init_state] = 0 + heuristic(init_state, problem)

  while not queue.isEmpty():
    state, trajectory = queue.pop()
    if state in seen:
      continue
    seen.add(state)
    if problem.isGoalState(state):
      return trajectory
    else:
      for successor, action, cost in problem.getSuccessors(state):
        if successor not in seen:
          g[successor] = g[state] + cost
          f[successor] = g[successor] + heuristic(successor, problem)
          queue.push((successor, trajectory + [action]), f[successor])
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch