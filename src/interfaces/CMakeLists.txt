#####################################
# Target setup
add_library(mavs_interfaces STATIC
    drive_px2.cpp
)
target_include_directories(mavs_interfaces
PUBLIC # USE (INTERFACE) and BUILD (PRIVATE) headers
	$<INSTALL_INTERFACE:include> # Uses this when installed
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Uses this when building
)
# Build the object library position independent
set_target_properties(mavs_interfaces PROPERTIES POSITION_INDEPENDENT_CODE True)

#####################################
# Target install
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS mavs_interfaces
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)