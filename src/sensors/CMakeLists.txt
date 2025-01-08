#####################################
# Target setup
add_library( mavs_sensor STATIC
io/json_io.cpp
io/applanix_io.cpp
io/user_io.cpp

annotation_colors.cpp
annotation.cpp

lidar/lidar.cpp
lidar/lidar_tools.cpp
lidar/planar_lidar.cpp
lidar/lms_291.cpp
lidar/vlp16.cpp
lidar/hdl32e.cpp
lidar/hdl64e.cpp
lidar/hdl64e_simple.cpp
lidar/hdl512e.cpp
lidar/m8.cpp
lidar/os1.cpp
lidar/os1_16.cpp
lidar/os2.cpp
lidar/robosense32.cpp
lidar/four_pi.cpp
lidar/anvel_api_lidar.cpp

camera/camera.cpp
camera/rgb_camera.cpp
camera/rccb_camera.cpp
camera/nir_camera.cpp
camera/lwir_camera.cpp
camera/micasense_red_edge.cpp
camera/simple_camera.cpp
camera/ortho_camera.cpp
camera/fisheye_camera.cpp
camera/distortion_model.cpp
camera/spherical_camera.cpp
camera/lens_drop.cpp
camera/add_rain_to_existing_image.cpp
camera/path_tracer.cpp
camera/oak_d_camera.cpp

gps/gps.cpp
gps/satellite.cpp

rtk/rtk.cpp

compass/compass.cpp

imu/imu.cpp
imu/imu_simple.cpp
imu/mems_sensor.cpp

radar/radar.cpp

occupancy_grid_detector/occupancy_grid_detector.cpp
object_detector/object_detector.cpp

ins/ins.cpp
ins/swiftnav_piksi.cpp
ins/swiftnav_msg.cpp
)
target_include_directories(mavs_sensor
PUBLIC # USE (INTERFACE) and BUILD (PRIVATE) headers
	$<INSTALL_INTERFACE:include> # Uses this when installed
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Uses this when building
PRIVATE # BUILD-only includes
	${CMAKE_CURRENT_SOURCE_DIR}
)
# Build the object library position independent
set_target_properties(mavs_sensor PROPERTIES POSITION_INDEPENDENT_CODE True)
target_link_libraries(mavs_sensor
PUBLIC
    mavs_interfaces
    mavs_raytracer
    mavs_core
    fastnoise
    rinex
    CImg::CImg
    glm::glm
PRIVATE
    tinyxml2
    TinyFileDialogs::TinyFileDialogs
    rapidjson
    cnpy
)

if (MAVS_USE_MPI)
    target_link_libraries(mavs_sensor PUBLIC MPI::MPI_CXX)
endif()
if (MAVS_USE_OMP)
    target_link_libraries(mavs_sensor PUBLIC OpenMP::OpenMP_CXX)
endif()

#####################################
# Target install
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS mavs_sensor
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)