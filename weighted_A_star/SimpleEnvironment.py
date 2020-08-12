import numpy
import math
import networkx as nx
from DiscreteEnvironment import DiscreteEnvironment
import matplotlib.pyplot as py
import matplotlib.patches as patches
class SimpleEnvironment(object):

	def __init__(self, resolution, dimension):

		graphFile = 'graphs/' + `dimension` + 'D.graphml'
		self.graph = nx.read_graphml(graphFile)
		# You will use this graph to query the discrete world it embeds in the space

		self.lower_limits = numpy.zeros(dimension)
		self.upper_limits = numpy.ones(dimension)
		self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

		obstacleFile = 'obstacles/' + `dimension` + 'D.txt'
		obstacles = []
		with open(obstacleFile) as file:
			for line in file:
				new_obstacle = [float(k) for k in line.split()]
				obstacles.append(new_obstacle)

		self.obstacles = numpy.array(obstacles)
		self.space_dim = dimension
		self.step_size = 0.01
		self.edg_count=0

	# Graph Methods
	def get_successors(self,v):
		'''
		@param v : vertex to expand
		returns the neighbours of vertex in graph
		'''
		# TODO: Here you will implement a function which
		# takes in the vertex ID and returns all of its successors
		# in the graph.
		successors = []
		c= self.discrete_env.nodeID_to_gridCoord(v)
		# successors = self.graph.neighbors(v)
		if self.space_dim == 2 :
			s1=self.discrete_env.gridCoord_to_nodeID([c[0]+1, c[1]])
			successors.append(s1)
			s2=self.discrete_env.gridCoord_to_nodeID([c[0]-1, c[1]])
			successors.append(s2)
			s3=self.discrete_env.gridCoord_to_nodeID([c[0], c[1]+1])
			successors.append(s3)
			s4=self.discrete_env.gridCoord_to_nodeID([c[0], c[1]-1])
			successors.append(s4)
		if self.space_dim == 3 :
			s1=self.discrete_env.gridCoord_to_nodeID([c[0]+1, c[1],c[2]])
			successors.append(s1)
			s2=self.discrete_env.gridCoord_to_nodeID([c[0]-1, c[1],c[2]])
			successors.append(s2)
			s3=self.discrete_env.gridCoord_to_nodeID([c[0], c[1]+1,c[2]])
			successors.append(s3)
			s4=self.discrete_env.gridCoord_to_nodeID([c[0], c[1]-1,c[2]])
			successors.append(s4)
			s5=self.discrete_env.gridCoord_to_nodeID([c[0], c[1],c[2]+1])
			successors.append(s5)
			s6=self.discrete_env.gridCoord_to_nodeID([c[0], c[1],c[2]-1])
			successors.append(s6)
		qualify=[]
		for node in successors:
			self.edg_count=self.edg_count+1
			if self.edge_validity_checker(node,v)==0:
				qualify.append(node)

		return qualify

	def state_validity_checker(self,v):
		'''
		@param v: the state which is to be checked for collision
		return: 0 if free and 1 if in collision
		'''
		# TODO: Here you will implement a function which
		# checks the collision status of a state represented
		# by the the vertex ID given
		c= self.discrete_env.nodeID_to_configuration(v)
		status = 0
		if self.space_dim == 2 :
			columndim = len(self.obstacles[:,0])
			a = [0]*columndim
			for i in range(columndim):
				a[i] = (self.obstacles[i,0]<=c[0]<=self.obstacles[i,2])&(self.obstacles[i,1]<=c[1]<=self.obstacles[i,3])
				if a[i]:
					status = 1
		if (self.space_dim == 3):
			columndim = len(self.obstacles[:,0])
			a = [0]*columndim
			for i in range(columndim):
				a[i] = (self.obstacles[i,0]<=c[0]<=self.obstacles[i,3])&(self.obstacles[i,1]<=c[1]<=self.obstacles[i,4])&(self.obstacles[i,2]<=c[2]<=self.obstacles[i,5])
				if (a[i]):
					status = 1
		return status

	def edge_validity_checker(self,u,v):
		'''
		@param u,v: the states between which the edge is to be checked for collision
		returns 0 if free and 1 if in collision
		'''
		# TODO: Here you will implement a function which
		# check the collision status of an edge between the states represented by the
		# vertex IDs, using `step_size` as the resolution factor.
		status = self.state_validity_checker(u)
		if status ==1:
			return status
		status = self.state_validity_checker(v)
		if status ==1:
			return status
		u = self.discrete_env.nodeID_to_configuration(u)
		v = self.discrete_env.nodeID_to_configuration(v)
		status = 0
		if (self.space_dim == 2):
			num=int(abs(u[0]-v[0])/self.step_size)
			low=min(u[0],v[0])
			x=numpy.zeros((num,2))
			for i in range(num):
				x[i,0]=low+i*self.step_size
				x[i,1]= (u[1]-v[1])/(u[0]-v[0])*(x[i,0]-v[0])+v[1]
				status=	self.state_validity_checker(self.discrete_env.configuration_to_nodeID(x[i:i+1][0]))
				if status == 1:
					return status
		if (self.space_dim == 3):
			num=int(abs(u[0]-v[0])/self.step_size)
			low=min(u[0],v[0])
			x=numpy.zeros((num,3))
			for i in range(num):
				x[i,0]=low+i*self.step_size
				x[i,1]= (u[1]-v[1])/(u[0]-v[0])*(x[i,0]-v[0])+v[1]
				x[i,2]=(u[2]-v[2])/(u[0]-v[0])*(x[i,0]-v[0])+v[2]
				status=	self.state_validity_checker(self.discrete_env.configuration_to_nodeID(x[i:i+1][0]))
				if status == 1:
					return status
		return status

	def compute_distance(self,u,v):
		'''
		@param u,v: the states between which euclidean distance is to be computed
		returns euclidean distance between nodes
		'''
		# TODO: Here you will implement a function which computes the
		# Euclidean distance between two states given their vertex IDs.
		#sum=0
		#dim=len(u)
		#for i on range(dim):
		#sum=sum+(u[i]-v[i])**2
		#	dist = sqrt(sum)
		#return dist
		c1 = self.discrete_env.nodeID_to_configuration(u)
		c2 = self.discrete_env.nodeID_to_configuration(v)
		c1 = numpy.array(c1)
		c2 = numpy.array(c2)
		return numpy.linalg.norm(c1-c2)

	def get_heuristic(self,v,t):
		'''
		@param v: node for which heuristic cost is to be determined
		@param t: target node
		returns the hueristic cost
		'''
		# TODO: Here you will implement a function to return the
		# heuristic cost of a state given its vertex ID and goal ID
		c1 = self.discrete_env.nodeID_to_configuration(v)
		c2 = self.discrete_env.nodeID_to_configuration(t)
		c1 = numpy.array(c1)
		c2 = numpy.array(c2)
		return numpy.linalg.norm(c1-c2)
	def plot(self,path):
		x= path[:,0]
		y= path[:,1]
		fig= py.figure()

		pa= fig.add_subplot(111,aspect='equal')
		for obs in self.obstacles:
			pa.add_patch(patches.Rectangle(
			(obs[0],obs[1]),
			obs[2]-obs[0],
			obs[3]- obs[1]
			))
        #py.plot([x[0] for x in plan],[x[1] for x in plan],'-r')
		py.plot(x,y)
		py.show()
	def smooth(self,current,next,last):
		next_distance= self.compute_distance(current,next)
		last_distance= self.compute_distance(current,last)
		nl_distance= self.compute_distance(last,next)
		if next_distance+last_distance == nl_distance:
			return 0
		else:
			return 1
