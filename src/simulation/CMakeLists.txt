#####################################
# Target setup
add_library( mavs_sim STATIC
    simulation.cpp
    simulation_loader.cpp
    ortho_viewer.cpp
    sensor_sim.cpp
)
target_include_directories(mavs_sim
PUBLIC # USE (INTERFACE) and BUILD (PRIVATE) headers
	$<INSTALL_INTERFACE:include> # Uses this when installed
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Uses this when building
)

# Build the object library position independent
set_target_properties(mavs_sim PROPERTIES POSITION_INDEPENDENT_CODE True)
target_link_libraries(mavs_sim
PUBLIC
    mavs_sensor
    mavs_raytracer
    mavs_driver
    mavs_vehicle
    mavs_core
    CImg::CImg
PRIVATE
    rapidjson
)

if(MAVS_USE_EMBREE)
    target_sources(mavs_sim PRIVATE rp3d_vehicle_viewer.cpp halo/halo_car.cpp)
    target_include_directories(mavs_sim
    PUBLIC # USE (INTERFACE) and BUILD (PRIVATE) headers
        $<INSTALL_INTERFACE:include> # Uses this when installed
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/halo/include> # Uses this when building
    )
    target_link_libraries(mavs_sim
    PUBLIC
        mavs_interfaces
    PRIVATE
        glm::glm
        TinyFileDialogs::TinyFileDialogs
    )
endif()

#####################################
# Target install
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS mavs_sim
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

#####################################
# Target setup
add_executable(mavs_simulation mavs_simulation.cpp simulation.cpp simulation_loader.cpp)
target_link_libraries(mavs_simulation
PUBLIC
    mavs_driver
    mavs_vehicle
    mavs_core
PRIVATE
    rapidjson
)
target_include_directories(mavs_simulation
PUBLIC # USE (INTERFACE) and BUILD (PRIVATE) headers
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Uses this when building
)
#####################################
# Target install
install(TARGETS mavs_simulation RUNTIME DESTINATION bin)

#####################################
# Target setup
add_executable(mavs_sensor_sim mavs_sensor_sim.cpp sensor_sim.cpp)
target_link_libraries(mavs_sensor_sim
PUBLIC
    mavs_sensor
    mavs_core
PRIVATE
    rapidjson
)
target_include_directories(mavs_sensor_sim
PUBLIC # USE (INTERFACE) and BUILD (PRIVATE) headers
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Uses this when building
)
#####################################
# Target install
install(TARGETS mavs_sensor_sim RUNTIME DESTINATION bin)
