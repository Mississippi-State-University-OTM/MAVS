import sys
# Set the path to the mavs python api, mavs.py
sys.path.append('/your/path/to/mavs/src/mavs_python')
# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths
# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path

# Select a dem file to load
dem_file = "/dems/example_ascii_dem.asc"

# create the MAVS DEM and load the file
dem = mavs.MavsDem()
dem.LoadEsriAscii(mavs_data_path+dem_file, interp_no_data = True)

# downsample by a factor of 2 and display
dem.Downsample(2)
dem.Display()

# save the results
dem.SaveAsObj("example_ascii_to_obj")
dem.SaveAsAscii("example_ascii_ds2.asc")