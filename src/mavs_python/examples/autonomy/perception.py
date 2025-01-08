"""
Slope-based obstacle detection on a grid. Works on lidar point clouds.

author: Chris Goodin, cgoodin@cavs.msstate.edu
"""

import numpy as np
import math
import autonomy
import mavs_interface as mavs
import matplotlib.pyplot as plt

class LidarGrid(autonomy.OccupancyGrid):
    def __init__(self,dimensions=None):
        super().__init__(dimensions)
        self.lowest = np.zeros(shape=(1,1))
        self.highest = np.zeros(shape=(1,1))
        self.height_thresh = 1.0
        self.plot = mavs.MavsPlot()
        self.path = []
        self.inflation = 0
        if dimensions:
            if len(dimensions) == 2:
                self.resize(dimensions[0],dimensions[1])
    def resize(self,width,height):
        """Resize the grid to width X height cells."""
        self.data = np.zeros(shape=(width,height))
        self.lowest = np.zeros(shape=(width,height))
        self.highest = np.zeros(shape=(width,height))
        self.lowest.fill(10000.0)
        self.highest.fill(-10000.0)
        self.info.width = width
        self.info.height = height
    def coordinate_to_index(self,x,y):
        """Convert a coordinate in local ENU to grid indices."""
        i = int(math.floor((x - self.info.origin.x) / self.info.resolution))
        j = int(math.floor((y - self.info.origin.y) / self.info.resolution))
        i = max(0,min(i,self.info.width-1))
        j = max(0,min(j,self.info.height-1))
        return [i,j]
    def index_to_coordinate(self,ix,iy):
        """Convert a grid index to local ENU."""
        x = ix * self.info.resolution + self.info.origin.x
        y = iy * self.info.resolution + self.info.origin.y
        return [x,y]
    def index_path_to_coordinates(self,path):
        """Convert a path listed in indices to local ENU."""
        path_enu = []
        if path:
            for p in path:
                point_enu = self.index_to_coordinate(p[0],p[1])
                path_enu.append(point_enu)
        return path_enu
    def coordinate_is_valid(self,idx):
        """Determine if a index coordinate is on the map."""
        if idx[0] >= 0 and idx[0] < self.info.width and idx[1] >= 0 and idx[1] < self.info.height:
            return True
        else:
            return False
    def set_origin(self,x,y):
        """Set the local origin in ENU. Lower-left corner of the grid."""
        self.info.origin.x = x
        self.info.origin.y = y
    def add_registered_points(self,points):
        """Add lidar points already registered to world coordinates."""
        for p in points:
            idx = self.coordinate_to_index(p[0],p[1])
            i = idx[0]
            j = idx[1]
            if (self.coordinate_is_valid(idx)):
                if (p[2]>self.highest[i,j]):
                    self.highest[i,j] = p[2]
                if (p[2]<self.lowest[i,j]):
                    self.lowest[i,j] = p[2]
                if self.highest[i][j]-self.lowest[i,j]>self.height_thresh:
                    self.data[i,j] = 1.0
                    if (self.inflation>0):
                        for iii in range(-self.inflation,self.inflation+1):
                            for jjj in range(-self.inflation,self.inflation+1):
                                ii = iii+i
                                jj = jjj+j
                                if ii>=0 and ii<self.info.width and jj>=0 and jj<self.info.height:
                                    self.data[ii,jj]=1.0
    def add_points(self,pos,quat,points):
        """Add lidar points in local coordinates
       
        Use the supplied position and orientation to register them to the grid.
        """

        q = Quaternion([quat[0],quat[1],quat[2],quat[3]])
        position = Vector3([pos[0],pos[1],pos[2]])
        for p in points:
            v = Vector3([p[0],p[1],p[2]])
            vprime = q.rotate(v)
            vprime = vprime + position
            idx = self.coordinate_to_index(vprime.x,vprime.y)
            i = idx[0]
            j = idx[1]
            if self.coordinate_is_valid(idx):
                if (v.z>self.highest[i,j]):
                    self.highest[i,j] = v.z
                if (v.z<self.lowest[i,j]):
                    self.lowest[i,j] = v.z
                if self.highest[i,j]-self.lowest[i,j]>self.height_thresh:
                    self.data[i,j] = 1.0
    def SetCurrentPath(self,path):
        """Set the current path, for visualization purposes."""
        self.path = path
    def display(self):
        """Display the current map and path."""
        colorplot = np.zeros(shape=(self.info.width,self.info.height,3))
        for i in range(self.info.width):
            for j in range(self.info.height):
                colorplot[i][j][0]=255*self.data[i][j]
        for p in self.path:
            if (p[0]>=0 and p[0]<self.info.width and p[1]>=0 and p[1]<self.info.height):
                colorplot[int(p[0])][int(p[1])][1] = 255
        self.plot.PlotColorMatrix(colorplot)

