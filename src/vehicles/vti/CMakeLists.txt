#####################################
# Target setup
add_library(mavs_vti STATIC
	fine_grained.cpp
	coarse_grained.cpp
	brixius.cpp
	nicolas_comstock.cpp
	spring_tire.cpp
	tire.cpp
)
target_include_directories(mavs_vti
PUBLIC # USE (INTERFACE) and BUILD (PRIVATE) headers
	$<INSTALL_INTERFACE:include> # Uses this when installed
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Uses this when building
PRIVATE # BUILD-only includes
	${CMAKE_CURRENT_SOURCE_DIR}
)
# Build the object library position independent
set_target_properties(mavs_vti PROPERTIES POSITION_INDEPENDENT_CODE True)

#####################################
# Target install
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS mavs_vti
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)