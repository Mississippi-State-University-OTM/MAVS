#####################################
# Target setup
add_library( mavs_driver STATIC
	simple_path_planner/simple_path_planner.cpp 
	simple_path_planner/astar.cpp 
	simple_waypoint_follower/simple_waypoint_follower.cpp
)
target_include_directories(mavs_driver
PUBLIC # USE (INTERFACE) and BUILD (PRIVATE) headers
	$<INSTALL_INTERFACE:include> # Uses this when installed
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Uses this when building
PRIVATE # BUILD-only includes
	${CMAKE_CURRENT_SOURCE_DIR}
)
# Build the object library position independent
set_target_properties(mavs_driver PROPERTIES POSITION_INDEPENDENT_CODE True)

target_link_libraries(mavs_driver
PUBLIC
	mavs_sensor
	mavs_core
	CImg::CImg
	glm::glm
PRIVATE
	rapidjson
)

#####################################
# Target install
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS mavs_driver
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
) 
