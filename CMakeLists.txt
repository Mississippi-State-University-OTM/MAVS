## framework for simulating autonomous vehicles
# https://github.com/pabloariasal/modern-cmake-sample/blob/master/libjsonutils/CMakeLists.txt
# -DBUILD_TESTING=OFF -DBUILD_SHARED_LIBS=OFF -DEMBREE_STATIC_LIB=ON -DBUILD_STATIC_LIBS=ON
# cmake .. -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=install -DMAVS_USE_EMBREE=ON --graphviz=graphdir/graph
# dot graphdir/graph -T png -o graph.png

cmake_minimum_required(VERSION 3.10)

project(msu_autonomous_vehicle_simulator
LANGUAGES
    CXX
VERSION 0.1.0
#DESCRIPTION "The MSU Autonomous Vehicle Simulator (MAVS) is a physics based vehicle simulation for off-road autonomy research."
#HOMEPAGE_URL "https://gitlab.com/cgoodin/msu_autonomous_vehicle_simulator"
)
cmake_policy(VERSION 3.10)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)

# Build options
option(MAVS_USE_EMBREE "Build embree-based ray-tracer" ON)
option(MAVS_USE_MPI "Build using MPI compilers for optimal performance" OFF)
option(RP3D_DOUBLE_PRECISION "Use double precision calculations with RP3D" OFF)
option(MAVS_USE_OMP "Enable multi-threading. Should not be used in combination with MPI" ON)
option(MAVS_USE_CHRONO "Enable vehicle dynamics simulation with Chrono, if it is installed" OFF)

if (MAVS_USE_CHRONO)
	SET(CHRONO_DATA_PATH "~" CACHE PATH "Full path to chrono/data/vehicle" )
	add_definitions(-DCHRONO_DATA_PATH=${CHRONO_DATA_PATH})
endif()
# Default data directory
SET(DEFAULT_DATA_DIR "~" CACHE PATH "Default location of MAVS data" )
add_definitions(-DDATA_PATH=${DEFAULT_DATA_DIR})

# setup dependencies
set(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX} ${CMAKE_MODULE_PATH})
include(cmake/deps.cmake)
add_subdirectory("extern")

set(BUILD_TESTING CACHE BOOL OFF)

if (WIN32)
    # TODO: Move to local disabling of warnings
    # Disable warnings about unsafe C in MSVC
    add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
    # Makes INSTALL a visible target
    SET(CMAKE_VS_INCLUDE_INSTALL_TO_DEFAULT_BUILD 1)
    # TODO: See if this is explicitly needed and can't be declared differently
	#SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
endif()

if (MAVS_USE_OMP)
add_definitions(-DUSE_OMP)
endif()

# subprojects
add_subdirectory("src") 

# examples
add_subdirectory("examples")
# utilities
add_subdirectory("utilities")

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    add_subdirectory("unit_tests")
endif()


#####################################
# Install Package Config Files
install(EXPORT mavs-targets
FILE
    MAVSTargets.cmake
NAMESPACE
    MAVS::
DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/MAVS
)

#Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/MAVSConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/MAVSConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/MAVSConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/MAVS
)

#Install the config, configversion and custom find modules
install(FILES
    #${CMAKE_CURRENT_LIST_DIR}/cmake/FindRapidJSON.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/MAVSConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/MAVSConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/MAVS
)

##############################################
## Add test
# enable ctest; Testing can be toggled with BUILD_TESTING
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    #include(CTest)
endif()