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
import sys

# Set the path to the mavs python api, mavs_interface.py
# you will have to change this on your system
sys.path.append(r'C:\Users\cgoodin\Desktop\code\mavs\src\mavs_python')

# Load the mavs python modules
import mavs_interface

# get the name of the file to add rain to
if (len(sys.argv)>1):
    fname = sys.argv[1]
else:
    print("ERROR: Must provide image file name on command line")
    exit(1)
# get the rain rate from command line
if (len(sys.argv)>2):
    rain_rate = float(sys.argv[2])
else:
    rain_rate = 5.0
# add rain to the image
mavs_interface.AddRainToImage(fname,rain_rate,add_drops=True)