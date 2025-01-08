#####################################
# Target setup
add_library(mavs_core STATIC
math/matrix.cpp
math/polygon.cpp
math/utils.cpp
math/segment.cpp
math/curve_fitter.cpp

environment/environment.cpp
environment/solar_position.cpp
environment/clouds.cpp

environment/snow/snowflake.cpp
environment/snow/snow.cpp
environment/fog/fog.cpp
environment/particle_system/particle.cpp
environment/particle_system/particle_system.cpp

environment/actors/actor.cpp

coordinate_systems/coord_conversions.cpp
coordinate_systems/ellipsoid.cpp

pose_readers/anvel_vprp_reader.cpp
pose_readers/waypoints.cpp
pose_readers/trail.cpp

plotting/mavs_plotting.cpp
plotting/map_viewer.cpp

terrain_generator/species.cpp
terrain_generator/heightmap.cpp
terrain_generator/surface.cpp
terrain_generator/random_surface.cpp
terrain_generator/grd_surface.cpp
terrain_generator/random_scene.cpp
terrain_generator/plant.cpp
terrain_generator/erosion_simulator.cpp
terrain_generator/ecosystem.cpp
terrain_generator/profile_2d.cpp
)
target_include_directories(mavs_core
PUBLIC # USE (INTERFACE) and BUILD (PRIVATE) headers
    $<INSTALL_INTERFACE:include> # Uses this when installed
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Uses this when building
)
# Build the object library position independent
set_target_properties(mavs_core PROPERTIES POSITION_INDEPENDENT_CODE True)
# circular dependency!!!
target_link_libraries(mavs_core
PUBLIC
    mavs_raytracer
    hosekwilkie
    fastnoise
    TinyFileDialogs::TinyFileDialogs
    CImg::CImg
    glm::glm
PRIVATE
    rapidjson
)
if (MAVS_USE_MPI)
    target_link_libraries(mavs_core PUBLIC MPI::MPI_CXX)
endif()
if (MAVS_USE_OMP)
    target_link_libraries(mavs_core PRIVATE OpenMP::OpenMP_CXX)
endif()

#####################################
# Target install
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS mavs_core
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)