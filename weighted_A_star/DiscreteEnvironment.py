import numpy

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution)

    def configuration_to_nodeID(self, config):

        # TODO: This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        coord = [0] * self.dimension
        for id in range(self.dimension):
            coord[id]= numpy.floor(round((config[id]/self.resolution),3))
        node_id=0
        accumulate=1
        gridlower_limits = self.configuration_to_gridCoord(self.lower_limits)
        for id in range(self.dimension):
            node_id = node_id+accumulate*abs(coord[id]-gridlower_limits[id])
            accumulate=accumulate*self.num_cells[id]
        return int(node_id)
    def accumulate(self,index):
	#present divisor calculations
	   accumulate=1
	   for id in range(self.dimension-index-1):
		      accumulate= accumulate*self.num_cells[id]
	   return int(accumulate)

    def nodeID_to_configuration(self, node_id):

        # TODO: This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        coord = [0] * self.dimension
        node_id = int(node_id)
        gridlower_limits = self.configuration_to_gridCoord(self.lower_limits)

        for id in range(self.dimension-1):
            accumulate = self.accumulate(id)
            coord[self.dimension-id-1]=node_id//accumulate + gridlower_limits[self.dimension-id-1]
            node_id=node_id%(accumulate)
        coord[0]=node_id+ gridlower_limits[0]

        config = [0] * self.dimension
        for id in range(self.dimension):
            config[id]=coord[id]*self.resolution
            config[id] = round(config[id], 3)
        return config

    def configuration_to_gridCoord(self, config):

        # TODO: This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [0] * self.dimension
        for id in range(self.dimension):
            coord[id]= numpy.floor(round((config[id]/self.resolution),3))
        return coord

    def gridCoord_to_configuration(self, coord):

        # TODO: This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        for id in range(self.dimension):
            config[id]=coord[id]*self.resolution
            config[id] = float(config[id])
        return config

    def gridCoord_to_nodeID(self,coord):

        # TODO: This function maps a grid coordinate to the associated
        # node id
        node_id=0
        accumulate=1
        gridlower_limits = self.configuration_to_gridCoord(self.lower_limits)
        for id in range(self.dimension):
            node_id = node_id+accumulate*abs(coord[id]-gridlower_limits[id])
            accumulate=accumulate*self.num_cells[id]
        return int(node_id)

    def nodeID_to_gridCoord(self, node_id):

        # TODO: This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
        node_id = int(node_id)
        gridlower_limits = self.configuration_to_gridCoord(self.lower_limits)

        for id in range(self.dimension-1):
            accumulate = self.accumulate(id)
            coord[self.dimension-id-1]=node_id//accumulate + gridlower_limits[self.dimension-id-1]
            node_id=node_id%(accumulate)
        coord[0]=node_id+ gridlower_limits[0]
        return coord
