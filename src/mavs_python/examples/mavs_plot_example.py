'''
Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

*NOTICE*
Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!

Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.

Copyright 2018 (C) Mississippi State University
'''
''' mavs_plot_example.py

Short example to demonstrate how the MAVS python API
can be used to rapidly update plots on a loop, something
pyplot isn't very good at.
'''
import sys
#path to the mavs python api
sys.path.append('/home/cgoodin/mavs/src/mavs_python')
import mavs_interface
import mavs_python_paths

plotter = mavs_interface.MavsPlot()
width = 512
height = 64
depth = 3
A = [[[0.0]*3]*height]*width

tmax = 100

for t in range(tmax):
    f = (1.0*t)/tmax
    for i in range(width):
        for j in range(height):
            A[i][j][0] = (1.0-f)*255.0
            A[i][j][1] = 255.0*f
            A[i][j][2] = (255.0*j)/(1.0*height)
    plotter.PlotColorMatrix(A)