"""
Potential Field based path planner
author: Atsushi Sakai (@Atsushi_twi)
Ref:
https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf

Original source code from:
https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/PotentialFieldPlanning\

Modified by CTG on 6-17-2020 to be faster and work with an Occupancy grid.
"""

import math

class PotentialFieldPlanner(object):
    def __init__(self):
        # Parameters
        self.KP = 5.0  # attractive potential gain
        self.ETA = 100.0  # repulsive potential gain
        self.obs_cutoff_dist = 20.0
    def hypot(self,x,y):
        return math.sqrt(x*x+y*y)
    def calc_attractive_potential(self, x, y, gx, gy):
        return 0.5 * self.KP * self.hypot(x - gx, y - gy)

    def calc_repulsive_potential(self,x, y, ox, oy):
        repul = 0.0
        for i, _ in enumerate(ox):
            d = self.hypot(x - ox[i], y - oy[i])
            repul = repul + self.ETA/(d*d+0.1)
        return repul

    def get_motion_model(self,step):
        motion = [[step, 0],
                  [0, step],
                  [-step, 0],
                  [0, -step],
                  [-step, -step],
                  [-step, step],
                  [step, -step],
                  [step, step]]
        return motion

    def potential_field_planning(self, grid, sx, sy, gx, gy, ox, oy):
        # search path
        minx = grid.info.origin.x
        miny = grid.info.origin.y
        reso = grid.info.resolution
        d = self.hypot(sx - gx, sy - gy)
        max_steps = math.floor(3.0*d)
        ix = round((sx - minx) / reso)
        iy = round((sy - miny) / reso)
        gix = round((gx - minx) / reso)
        giy = round((gy - miny) / reso)
        rx, ry = [sx], [sy]
        xp = sx
        yp = sy
        motion = self.get_motion_model(reso)
        nsteps = 0
        while d >= reso and nsteps<max_steps:
            minp = float("inf")
            mindx = reso
            mindy = 0.0
            for i, _ in enumerate(motion):
                dx = reso*motion[i][0]
                dy = reso*motion[i][1]
                xx = xp+dx
                yy = yp+dy
                ug = self.calc_attractive_potential(xx, yy, gx, gy)
                uo = self.calc_repulsive_potential(xx, yy, ox, oy)
                p = ug + uo
                if minp > p:
                    minp = p
                    mindx = dx
                    mindy = dy
            xp = xp+mindx
            yp = yp+mindy
            d = self.hypot(gx - xp, gy - yp)
            rx.append(xp)
            ry.append(yp)
            nsteps = nsteps+1
        return rx, ry

    def plan(self,grid,sx,sy,gx,gy):
        """ Update the field calculation and return the planned path.
        
        params:
        grid (OccupancyGrid): Current map
        sx (float): Current x-coordinate of the vehicle in local ENU
        sy (float): Current y-coordinate of the vehicle in local ENU
        gx (float): Current x-coordinate of the goal-point in local ENU
        gy (float): Current y-coordinate of the goal-point in local ENU
        """

        ox = []
        oy = []
        for i in range(grid.info.width):
            x = grid.info.origin.x + (i+0.5)*grid.info.resolution
            dx = sx-x
            for j in range(grid.info.height):
                y = grid.info.origin.y + (j+0.5)*grid.info.resolution
                dy = sy-y
                if grid.data[i,j]>0:
                    d = math.sqrt(dx*dx+dy*dy)
                    if d<self.obs_cutoff_dist:
                        ox.append(x)
                        oy.append(y)
        # path generation
        rx,ry = self.potential_field_planning(grid, sx, sy, gx, gy, ox, oy)
        path_enu = []
        path = []
        for i in range(len(rx)):
            path_enu.append([rx[i],ry[i]])
            idx_i = math.floor((rx[i] - grid.info.origin.x) / grid.info.resolution)
            idx_j = math.floor((ry[i] - grid.info.origin.y) / grid.info.resolution)
            path.append([idx_i,idx_j])
        return path, path_enu


