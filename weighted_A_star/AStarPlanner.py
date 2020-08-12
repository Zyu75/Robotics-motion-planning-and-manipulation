import sys
import time
import numpy
import IPython
import SimpleEnvironment as SiE
from PriorityQueue import PriorityQueue
import copy
import time
import math

class AStarPlanner(object):
    def __init__(self, planning_env):
        self.PrQ = PriorityQueue()
        self.planning_env = planning_env
        self.nodes = dict()


    def Plan(self, start_config, goal_config):


        print start_config
        print goal_config

        cost_so_far = {}
        start= self.planning_env.discrete_env.configuration_to_nodeID(start_config)
        goal= self.planning_env.discrete_env.configuration_to_nodeID(goal_config)
        print "start:",start
        print "goal:",goal
        count=0
        alongtheway={}
        cost_so_far[start] = 0
        alongtheway[start]=0
        self.PrQ.put(start,0)
        weight=5

        while not self.PrQ.empty():
            current= self.PrQ.get()
            count = count +1
            if (current == goal):
                # print 'alongtheway=', alongtheway
                print "reach goal"
                parent=copy.copy(goal)
                path= numpy.array([self.planning_env.discrete_env.nodeID_to_configuration(parent)])
                nodepath= numpy.array([parent])
                parent= alongtheway[parent]
                while parent!=0:

                    path= numpy.vstack((numpy.array(self.planning_env.discrete_env.nodeID_to_configuration(parent)),path))
                    nodepath= numpy.vstack((numpy.array(parent),nodepath))
                    parent = alongtheway[parent]
                print "nodes expanded:",count
                print "edg check:",self.planning_env.edg_count
                print "Final cost:",cost_so_far[goal]
                print nodepath
                # if self.planning_env.space_dim==2:
                #     self.planning_env.plot(path)
                return path

            for node in self.planning_env.get_successors(current):
                n_g =  cost_so_far[current] +self.planning_env.compute_distance(current, node)
                # + self.planning_env.smooth(current,node,alongtheway[current])

                if node not in cost_so_far or n_g < cost_so_far[node]:

                    cost_so_far[node] = n_g
                    n_f = n_g +weight*self.planning_env.get_heuristic(node,goal)
                    # print 'node', node
                    # print 'n_f', n_f
                    self.PrQ.put(node,n_f)
                    alongtheway[node]= current

        print "Path Not Found"
        return 0

        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of alongtheway
        #  and n is the dimension of the robots configuration space

if __name__ == '__main__':

    l= SiE.SimpleEnvironment(0.025,3)
    d= AStarPlanner(l)

    source_config = numpy.ones(3)*0.1
    target_config = numpy.ones(3)*0.9
    plan = d.Plan(source_config, target_config)
