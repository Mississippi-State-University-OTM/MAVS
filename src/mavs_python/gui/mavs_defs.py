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
def rgb_to_hex(r,g,b):
    return '#%02x%02x%02x'%(r,g,b)

#see color palette defs from
#www.opa.msstate.edu/identity/assets/msstate_colors.pdf
global maroon, darkgrey, lightgrey
maroon = rgb_to_hex(102,0,0)
darkgrey = rgb_to_hex(117,120,123)
lightgrey = rgb_to_hex(204,204,204)
gold = rgb_to_hex(255,184,28)


global data_path_

camera_models =  ["Flea3-4mm","XCD-V60", "MachineVision", "HD1080"]

lidar_models =  ["HDL-64E","HDL-32E", "VLP-16","M8","LMS-291"]

radar_models = ['Delphi Long Range', 'Delphi Mid Range']

sensor_models = ("HDL-64E","HDL-32E", "VLP-16","M8","LMS-291",
                 "Flea3-4mm","XCD-V60","Delphi Long Range", "Delphi Mid Range", "Default IMU", "")
sensor_model_types = {'HDL-64E':'lidar','HDL-32E':'lidar','VLP-16':'lidar',
                  'M8':'lidar','LMS-291':'lidar','Flea3-4mm':'camera',
                      'XCD-V60':'camera', 'Delphi Long Range':'radar', 
                      'Delphi Mid Range':'radar', 'Default IMU':'imu'}
sensor_types = ('lidar','camera','compass','gps','fisheye','radar','imu','')
