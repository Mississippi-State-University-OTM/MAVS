# Frequently Asked Questions (FAQ)

### Q: After building MAVS on Ubuntu, I get an Rp3d error message when trying to run examples with Rp3d?
If you built MAVS on Ubuntu or another Linux/Unix system using gcc, and you get the following error message when you try to run a simulation:
```
reactphysics3d::SliderJoint::SliderJoint(reactphysics3d::uint, const reactphysics3d::SliderJoinInfo&): Assertion 'mUpperLimit >= decimal(0.0)' failed
```
this is caused by not setting the "CMAKE_CONFIGURATION_TYPES" variable correctly when configuring the build with CMake. Make sure the value is set to "Release".

### Q: After building MAVS on Windows, I get linker errors related to reactphysics3d.lib?
In your CMake dialog box, uncheck "BUILD_SHARED_LIBS", reconfigure, and rebuild.

### Q: I get the error "No module namved mavs_interface" when trying to run python examples.
You need to add *mavs_interface.py* to your python search path.
```python
import sys
sys.path.append(r'C:/your/path/to/mavs/src/mavs_python')
```
Note that you should probably change any "\" to "/". Many systems also experience errors if there is a space in the file path, so try to install MAVS in a directory path with no spaces.

### Q: When running MAVS-Python, Python can't find mavs.dll
If you are running MAVS with the Python interface and you get an error message that says something similar to 
```
FileNotFoundError: Could not find module 'C:\path\to\mavs\build\lib\mavs.dll'
```
First verify that *mavs.dll* is indeed in the specified folder. If it is, make sure that you have [modified your system path to include the *mavs.dll* file](./InstallingMavsBinaries.md).