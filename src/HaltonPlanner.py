import math
import numpy
from matplotlib import pyplot as plt
import cv2
import Utils
import time
import random
import heapq
import copy

class HaltonPlanner(object):
  
  # planningEnv: Should be a HaltonEnvironment
  def __init__(self, planningEnv):
    self.planningEnv = planningEnv

  # Generate a plan
  # Assumes that the source and target were inserted just prior to calling this
  # Returns the generated plan
  def plan(self):
    self.sid = self.planningEnv.graph.number_of_nodes() - 2 # Get source id
    self.tid = self.planningEnv.graph.number_of_nodes() - 1 # Get target id

    self.closed = {} # The closed list, or visited notes
    self.parent = {self.sid:None} # A dictionary to store the graph connection for our selected path
    self.open = {self.sid: 0 + self.planningEnv.get_heuristic(self.sid, self.tid)} # The open list, or frontier
    self.gValues = {self.sid:0} # accumulation cost of path without heuristic
    self.planIndices = []
    self.cost = 0
    # ------------------------------------------------------------
    # YOUR CODE HERE
    # 
    # Implement A*
    # Functions that you will probably use
    # - self.get_solution()
    # - self.planningEnv.get_successors() - returns all adjacent nodes of a given node
    # - self.planningEnv.get_distance() - get distance between 2 vertices/nodes
    # - self.planningEnv.get_heuristic() - get euclidean distance from source node to target node
    # Note that each node in the graph has both an associated id and configuration
    # You should be searching over ids, not configurations. get_successors() will return
    #   the ids of nodes that can be reached. Once you have a path plan
    #   of node ids, get_solution() will compute the actual path in SE(2) based off of
    #   the node ids that you have found.
    #-------------------------------------------------------------
    #each node's id is its index in that graph to identify that node, the graph has roughly 1250 nodes (set by halton_points)
    #get_solution will take the last vid in our path and return a list of [x,y] poses of vertices
    #self.parent stores the final selected that will be used later on in get_solution
    #get_solution will backtrack retrieve the whole path with the last vid given
    #e.g. self.parent = {1:2, 2:3, 3:4, 4:None} 1's parent is 2, 2's parent is 4, etc. the last node's parent MUST be
    #None cuz just like list data structure's leaf node (self.sid:None is already set above)
    
    # prioritized BFS: <always select the node from current frontier with the shortest h+g path to expand
    # plan is found as soon as a frontier node touches the target>
    parents = {self.sid: {self.sid:None}} #a dict of dict contains path history to the every frontier node
    
    while self.open:
      openq = []
      openlist = [(item[1],item[0]) for item in self.open.items()] #heapq sort based on the 1st val of each tuple element
      for h in openlist:
        heapq.heappush(openq, h)
      sel = heapq.heappop(openq) #sel[0]=g+h, sel[1]=id
      if sel[1] == self.tid:
        self.parent = parents[self.tid]
        break
      a = self.open.pop(sel[1])
      self.closed[sel[1]] = sel[0]
      adj = self.planningEnv.get_successors(sel[1]) #adjacent nodes
      for node in adj:
        if node in self.closed:
          continue
        #edge collision check
        parentConfig = self.planningEnv.get_config(sel[1])
        adjConfig = self.planningEnv.get_config(node) #current adjacent node
        if not self.planningEnv.manager.get_edge_validity(parentConfig, adjConfig):
          continue
        g = self.gValues[sel[1]] + self.planningEnv.get_distance(sel[1], node)
        h = self.planningEnv.get_heuristic(node, self.tid)
        f = g + h
        if node in self.gValues:
          if g > self.gValues[node]:
            continue
        self.open[node] = f
        self.gValues[node] = g
        pre_path = copy.deepcopy(parents[sel[1]])
        pre_path[node] = sel[1]
        parents[node] = pre_path

    return self.get_solution(self.tid) #here we return get_solution(selected_indices)

  # Try to improve the current plan by repeatedly checking if there is a shorter path between random pairs of points in the path
  def post_process(self, _plan, timeout): #uncomment this functon from PlannerNode to use

    t1 = time.time()
    elapsed = 0
    plan = copy.deepcopy(_plan)
    while elapsed < timeout: # Keep going until out of time
      # ---------------------------------------------------------
      # YOUR CODE HERE
      
      # Pseudocode
      
      # Pick random id i
      # Pick random id j
      # Redraw if i == j
      # Switch i and j if i > j
     
      # if we can find path between i and j (Hint: look inside ObstacleManager.py for a suitable function)
        # Get the path
        # Reformat the plan such that the new path is inserted and the old section of the path is removed between i and j
        # Be sure to CAREFULLY inspect the data formats of both the original plan and the plan returned
        # to ensure that you edit the path correctly
      i, j = 0, 0
      while i == j:
        i, j = random.randint(0, len(plan)-1), random.randint(0, len(plan)-1)
      if i > j:
        t = i
        i = j
        j = t
      config_i = [plan[i][0], plan[i][1]]
      config_j = [plan[j][0], plan[j][1]]
      if self.planningEnv.manager.get_edge_validity(config_i, config_j):
        new_plan = []
        for idx in range(0, i): 
          new_plan.append(plan[idx])
        new_plan = list(new_plan)
        px, py, clen = self.planningEnv.manager.discretize_edge(config_i, config_j)
        path_replacement = [list(a) for a in zip(px, py)]
        for pr in path_replacement:
          new_plan.append(pr)
        for idx in range(j, len(plan)): 
          new_plan.append(plan[idx])
        plan = numpy.array(new_plan)

      elapsed = time.time() - t1
    return plan

  # Backtrack across parents in order to recover path
  # vid: The id of the last node in the graph
  def get_solution(self, vid):

    # Get all the node ids
    planID = []
    while vid is not None:
      planID.append(vid)
      vid = self.parent[vid]

    plan = []
    planID.reverse()
    for i in range(len(planID) - 1):
      startConfig = self.planningEnv.get_config(planID[i])
      goalConfig = self.planningEnv.get_config(planID[i + 1])
      #startConfig is start vertex [x,y] pose, goalConfig is ... pose for next vertex we're going to
      px, py, clen = self.planningEnv.manager.discretize_edge(startConfig, goalConfig)
      plan.append([list(a) for a in zip(px, py)])
      self.planIndices.append(len(plan))
      self.cost += clen

    flatPlan = [item for sublist in plan for item in sublist]
    print('Path length of the current plan is: ', len(flatPlan))
    return flatPlan

  # Visualize the plan
  def simulate(self, plan):
    # Get the map
    envMap = 255*(self.planningEnv.manager.mapImageBW+1) # Hacky way to get correct coloring
    envMap = cv2.cvtColor(envMap, cv2.COLOR_GRAY2RGB)
    
    for i in range(numpy.shape(plan)[0]-1): # Draw lines between each configuration in the plan
      startPixel = Utils.world_to_map(plan[i], self.planningEnv.manager.map_info)
      goalPixel = Utils.world_to_map(plan[i+1], self.planningEnv.manager.map_info)
      cv2.line(envMap,(startPixel[0],startPixel[1]),(goalPixel[0],goalPixel[1]),(255,0,0),5)

    # Generate window
    cv2.namedWindow('Simulation', cv2.WINDOW_NORMAL)
    cv2.imshow('Simulation', envMap)

    # Terminate and exit elegantly
    cv2.waitKey(20000)
    cv2.destroyAllWindows()
