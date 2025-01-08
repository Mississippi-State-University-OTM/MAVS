import time
import sys
# Set the path to the mavs python api, mavs.py
sys.path.append('/your/path/to/mavs/src/mavs_python')
# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths
# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path

# load waypoints from a .vprp file
waypoints = mavs.MavsWaypoints()
waypoints.Load(mavs_data_path+"/waypoints/gmae_outer_loop.vprp");
waypoints.SaveAsJson("gmae_outer_loop.json")

new_waypointws = mavs.MavsWaypoints()
waypoints.LoadJson("gmae_outer_loop.json")
print(waypoints.waypoints[0])
print(waypoints.waypoints[-1])