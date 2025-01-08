# MAVS - C++
MAVS can be used as a C++ dependency and can be easily linked against using CMake.

## Adding MAVS to a CMake Project
MAVS can be found and imported by CMake using a `find_package(MAVS REQUIRED)` call in your CMakeLists.txt. Note that CMake 3.13+ is required. 

On Linux systems (e.g. Ubuntu), the standard installation location for MAVS is */usr/local*. Installing in */user/local* requires adminstrator (sudo) privileges to be invoked when running installation commands.

If MAVS was installed in a non-standard location, direct CMake to the appropriate directory by adding a `-DCMAKE_PREFIX_PATH=<path_to_MAVS>` to your CMake configure command (`cmake ..`). If Embree was also installed to a non-standard location, ensure that location is also included in CMAKE_PREFIX_PATH.

## Linking MAVS
Once `find_package` succeeds in finding MAVS, MAVS can be linked against via the `MAVS::mavs` target. This target carries all needed usage requirements for MAVS. MAVS API's can then be included and used as normal in the linking target's source code. Any needed includes, compile definitions, and linked libraries are handled by CMake automatically.

## MAVS C++ Example CMake Project
The following minimal CMake project imports MAVS and links a basic executable against it. The projects structure consists of two files, the `CMakeLists.txt` and the source file, `foo.cpp`, of the executable. Additionally, two empty directories, `build` and `install` are present to hold build and install artifacts (makefiles, binaries, etc.), respectively.
```bash
example_cmake_project
├── build
├── CMakeLists.txt
├── foo.cpp
└── install
```

To build the project, run the following in the build directory:
```bash
cmake ..                                 \
-DCMAKE_PREFIX_PATH=<MAVS_INSTALL_PREFIX> \
-DCMAKE_INSTALL_PREFIX=../install
```
If MAVS was installed to a non-default location on your system (i.e. not `/usr/local` on Unix or `C:\Program Files on Windows`), add the install prefix to CMAKE_PREFIX_PATH.
### CMakeLists.txt
The CMakeLists contains the following:
```cmake
cmake_minimum_required(VERSION 3.13)
project(example
VERSION 1.0.0
LANGUAGES CXX)

######################################################
####             Dependencies                     ####
######################################################
# This setups up some CMake variables for us when installing
include(GNUInstallDirs)

# MAVS will find all the dependencies it needs to function
find_package(MAVS REQUIRED)

######################################################
####                Targets                       ####
######################################################
# Basic executable
add_executable(example foo.cpp)
# Specify header includes
target_include_directories(example
PRIVATE # BUILD-only includes
    #${CMAKE_CURRENT_SOURCE_DIR}/include
)
# All that's needed to use MAVS is to link against it!
target_link_libraries(example PRIVATE MAVS::mavs)

######################################################
####            INSTALL STUFF                     ####
######################################################
install(TARGETS example
LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
```

The lines needed to use MAVS are the line to import MAVS
```cmake
find_package(MAVS REQUIRED)
```
and the line to link the executable against MAVS.
```cmake
target_link_libraries(example PRIVATE MAVS::mavs)
```

### foo.cpp
The example source file contains a minimal example to demonstrate inclusion and use of MAVS:
```c++
#include <sensors/lidar/hdl64e.h>
int main(int argc, char **argv){
    mavs::sensor::lidar::Hdl64E lidar;
    lidar.SetMode(0);
    return 0;
}
```
