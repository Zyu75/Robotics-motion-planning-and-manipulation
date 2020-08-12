import numpy
from DiscreteEnvironment import DiscreteEnvironment
import copy
import math
import argparse, openravepy, time
from HerbRobot import HerbRobot

class HerbEnvironment(object):

    def __init__(self, herb, resolution):

        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)
        self.step_size = 0.05
        self.space_dim=len(self.lower_limits)
        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.gridCoord_to_configuration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')

        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7],
                                  [-1, 0,  0, 0],
                                  [ 0, 1,  0, 0],
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        self.edg_count=0

    def get_successors(self, node_id):

        # TODO: Here you will implement a function that returns all the
        # neighbouring configurations' node IDs of the configuration
        # represented by the input `node_id`
        successors1=[0] * self.space_dim
        successors2=[0] * self.space_dim
        grid_coord = self.discrete_env.nodeID_to_gridCoord(node_id)

        for id in range(self.space_dim):
            n1= copy.copy(grid_coord)
            n2= copy.copy(grid_coord)
            n1[id]= grid_coord[id]+1
            n2[id]= grid_coord[id]-1
            successors1[id]=self.discrete_env.gridCoord_to_nodeID(n1)
            successors2[id]=self.discrete_env.gridCoord_to_nodeID(n2)
        successors1.extend(successors2)
        successors = [item for item in successors1 if item >= 0]

        node_check=[]
        for node in successors:
            self.edg_count=self.edg_count+1  # the number of nodes expanded to.
            if self.state_validity_checker(node)==0:
                node_check.append(node)
        qualify=[]
        for node in node_check:
            if self.edge_validity_checker(node_id,node)==0:
                qualify.append(node)
        return qualify
    def state_validity_checker(self, node_id):

        # TODO: Here you will implement a function which
        # checks the collision status of a state represented
        # by the the node ID given
        status=0
        conf = self.discrete_env.nodeID_to_configuration(node_id)
        limits= self.robot.GetActiveDOFLimits();
        for i in range(0,len(conf)):
            if conf[i]<limits[0][i] or conf[i]>limits[1][i]:
                status = 1
                return status

        with self.robot:
            self.robot.SetActiveDOFValues(conf)
            status_check = self.robot.CheckSelfCollision() or self.robot.GetEnv().CheckCollision(self.robot)
        if status_check==True:
            status = 1
            return status
        return status

    def edge_validity_checker(self, start_id, goal_id):
        # TODO: Here you will implement a function which
        # check the collision status of an edge between the
        # states represented by the node IDs given using
        # using `step_size` as the resolution factor.
        # status = 0
        # u_config= numpy.asarray(self.discrete_env.nodeID_to_configuration(start_id))
        # v_config= numpy.asarray(self.discrete_env.nodeID_to_configuration(goal_id))
        #
        # steps = numpy.arange(0.0, 1.0, self.step_size)
        # for step in steps.tolist():
        #     n = v_config*step +(1-step)*u_config
        #
        #
        #     status = self.state_validity_checker(self.discrete_env.configuration_to_nodeID(n))
        #     # print 'status1=', status
        #     if status == 1:
        #         # print 'status2=', status
        #         status = 1
        #         return status
        # return status



        u_config= numpy.asarray(self.discrete_env.nodeID_to_configuration(start_id))
        v_config= numpy.asarray(self.discrete_env.nodeID_to_configuration(goal_id))
        start_config_re = copy.copy(u_config)
        ratio = float(self.step_size)/float(self.compute_distance(start_id, goal_id))
        while self.compute_distance(self.discrete_env.configuration_to_nodeID(start_config_re), goal_id) > self.step_size:
            for i in range(len(self.lower_limits)):
                start_config_re[i] = start_config_re[i] + ratio*(v_config[i]-start_config_re[i])
                if self.state_validity_checker(self.discrete_env.configuration_to_nodeID(start_config_re)) == 1:
                    return 1
        if self.compute_distance(self.discrete_env.configuration_to_nodeID(start_config_re), goal_id) < self.step_size:
            return 0

    def compute_distance(self, start_id, end_id):

        # TODO: Here you will implement a function which computes the
        # distance between two states given their node IDs.
        # u_state = numpy.asarray(self.discrete_env.nodeID_to_gridCoord(start_id))
        # #print "ustate:",u_state
        # v_state = numpy.asarray(self.discrete_env.nodeID_to_gridCoord(end_id))
        # #print "vstate:",v_state
        c1 = self.discrete_env.nodeID_to_configuration(start_id)
        c2 = self.discrete_env.nodeID_to_configuration(end_id)
        c1 = numpy.array(c1)
        c2 = numpy.array(c2)
        return numpy.linalg.norm(c1-c2)

    def get_heuristic(self, start_id, goal_id):

        # TODO: Here you will implement a function which computes the
        # distance between two states given their node IDs.
        c1 = self.discrete_env.nodeID_to_configuration(start_id)
        c2 = self.discrete_env.nodeID_to_configuration(goal_id)
        c1 = numpy.array(c1)
        c2 = numpy.array(c2)
        return numpy.linalg.norm(c1-c2)
    def smooth(self,current,next,last):
		next_distance= self.compute_distance(current,next)
		last_distance= self.compute_distance(current,last)
		nl_distance= self.compute_distance(last,next)
		if next_distance+last_distance == nl_distance:
			return 0
		else:
			return 1

