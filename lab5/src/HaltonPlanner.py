import math
import random
import time

import cv2
import numpy
import Utils
from matplotlib import pyplot as plt


class HaltonPlanner(object):
    # planningEnv: Should be a HaltonEnvironment
    def __init__(self, planningEnv):
        self.planningEnv = planningEnv

    # Generate a plan
    # Assumes that the source and target were inserted just prior to calling this
    # Returns the generated plan
    def plan(self):
        t1 = time.time()

        self.sid = self.planningEnv.graph.number_of_nodes() - 2  # Get source id
        self.tid = self.planningEnv.graph.number_of_nodes() - 1  # Get target id

        self.closed = {}  # The closed list
        self.parent = {self.sid: None}  # A dictionary mapping children to their parents
        self.open = {self.sid: 0 + self.planningEnv.get_heuristic(self.sid, self.tid)}  # The open list
        self.gValues = {self.sid: 0}  # A mapping from node to shortest found path length to that node
        self.planIndices = []
        self.cost = 0
        # YOUR CODE HERE
        while self.open:
            nid = min(self.open, key=self.open.get)
            for cid in self.planningEnv.get_successors(nid):
                if cid in self.closed:
                    continue
                if not self.planningEnv.manager.get_edge_validity(
                        self.planningEnv.get_config(nid), self.planningEnv.get_config(cid)):
                    continue

                g_val = self.gValues[nid] + self.planningEnv.get_distance(nid, cid)
                if cid not in self.open or g_val < self.gValues[cid]:
                    self.parent[cid] = nid

                    # target reached
                    if cid == self.tid:
                        self.open[cid] = g_val
                        plan = self.get_solution(cid)
                        plan = self.post_process(plan, 0.5)
                        print("Cost: ", self.cost)
                        print("Plan Indices: ", self.planIndices)
                        print("Plan Length: ", len(plan))
                        print("Time: ", time.time() - t1)
                        # self.simulate(plan)
                        return plan

                    self.gValues[cid] = g_val
                    self.open[cid] = g_val + self.planningEnv.get_heuristic(cid, self.tid)
            self.open.pop(nid)
            self.closed[nid] = 1

        return []

    # Try to improve the current plan by repeatedly checking if there is a shorter path between random pairs of points in the path
    def post_process(self, plan, timeout):
        t1 = time.time()
        elapsed = 0

        len_plan = len(plan)
        while elapsed < timeout:  # Keep going until out of time
            # YOUR CODE HERE
            i, j = numpy.random.randint(len_plan, size=2)
            if i == j:
                continue
            if i > j:
                i, j = j, i

            startConfig, endConfig = plan[i], plan[j]
            if self.planningEnv.manager.get_edge_validity(startConfig, endConfig):
                list_x, list_y, _ = self.planningEnv.manager.discretize_edge(startConfig, endConfig)
                plan[i:j] = [list(a) for a in zip(list_x, list_y)]
                len_plan = len(plan)

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
            px, py, clen = self.planningEnv.manager.discretize_edge(startConfig, goalConfig)
            plan.append([list(a) for a in zip(px, py)])
            self.planIndices.append(len(plan))
            self.cost += clen

        flatPlan = [item for sublist in plan for item in sublist]
        return flatPlan

    # Visualize the plan
    def simulate(self, plan):
        # Get the map
        envMap = 255*(self.planningEnv.manager.mapImageBW+1)  # Hacky way to get correct coloring
        envMap = cv2.cvtColor(envMap, cv2.COLOR_GRAY2RGB)

        for i in range(numpy.shape(plan)[0]-1):  # Draw lines between each configuration in the plan
            startPixel = Utils.world_to_map(plan[i], self.planningEnv.manager.map_info)
            goalPixel = Utils.world_to_map(plan[i+1], self.planningEnv.manager.map_info)
            cv2.line(envMap, (startPixel[0], startPixel[1]), (goalPixel[0], goalPixel[1]), (255, 0, 0), 5)

        # Generate window
        cv2.namedWindow('Simulation', cv2.WINDOW_NORMAL)
        cv2.imshow('Simulation', envMap)

        # Terminate and exit elegantly
        cv2.waitKey(20000)
        cv2.destroyAllWindows()
