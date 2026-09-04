add_subdirectory(drivers)
add_subdirectory(mavs_core)
add_subdirectory(raytracers)
add_subdirectory(sensors)
add_subdirectory(simulation)
add_subdirectory(vehicles)
add_subdirectory(interfaces)
# additional utilities
#add_subdirectory(utilities)
# unit tests

# examples
#add_subdirectory(examples)
# Python API
add_subdirectory(mavs_python)

#####################################
# Target setup
add_library(mavs SHARED mavs_c_defs.cpp mavs_c_defs.h)
target_link_libraries(mavs
PUBLIC
	mavs_sim # mavs and mavs_sim are kind of redundant if PIC is set
)
if (NOT MSVC)
	# Avoid a runtime dependency on the build machine's libstdc++/libgcc_s:
	# a newer build toolchain can produce a libmavs.so that references
	# symbols an older target system's libstdc++.so.6 doesn't have, and
	# even bundling a compatible copy alongside it doesn't reliably help,
	# since the dynamic loader reuses whichever libstdc++.so.6 the process
	# (e.g. Python) already loaded first.
	target_link_options(mavs PRIVATE -static-libstdc++ -static-libgcc)
endif()
target_include_directories(mavs
PUBLIC # USE (INTERFACE) and BUILD (PRIVATE) headers
	$<INSTALL_INTERFACE:include> # Uses this when installed
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> # Uses this when building
	$<INSTALL_INTERFACE:include/reactphysics3d> # This is a hack
)

# MAVS MATLAB interace library

add_library(mavs_matlab SHARED mavs_matlab_interface.cpp mavs_matlab_interface.h)
if (WIN32)
	set_property(TARGET mavs_matlab PROPERTY POSITION_INDEPENDENT_CODE OFF)
else()
	set_property(TARGET mavs_matlab PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()

target_link_libraries(mavs_matlab 
PUBLIC
	mavs_sim # mavs and mavs_sim are kind of redundant if PIC is set
)
target_include_directories(mavs_matlab
PUBLIC # USE (INTERFACE) and BUILD (PRIVATE) headers
	$<INSTALL_INTERFACE:include> # Uses this when installed
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> # Uses this when building
	$<INSTALL_INTERFACE:include/reactphysics3d> # This is a hack
)


#add_library(mavs_static STATIC mavs_c_defs.cpp mavs_c_defs.h)
#target_link_libraries(mavs_static
#PRIVATE
#	mavs_sim # mavs and mavs_sim are kind of redundant if PIC is set
#)
#target_include_directories(mavs_static
#PUBLIC # USE (INTERFACE) and BUILD (PRIVATE) headers
#	$<INSTALL_INTERFACE:include> # Uses this when installed
#	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> # Uses this when building
#)

#####################################
# Target install
install(FILES mavs_c_defs.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS mavs mavs_matlab
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

#add_library(mavs::mavs_static ALIAS mavs_static)
add_library(MAVS::mavs ALIAS mavs)
add_library(MAVS::mavs_matlab ALIAS mavs_matlab)


