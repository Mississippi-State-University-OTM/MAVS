add_subdirectory("vti")

#####################################
# Target setup
if (MAVS_USE_CHRONO)
add_library(mavs_vehicle STATIC
	autonomous_vehicle.cpp
	#turtle/turtle.cpp 
	#bicycle/bicycle.cpp 
	#car/full_car.cpp
	controllers/pid_controller.cpp
	controllers/pure_pursuit_controller.cpp
	rp3d_veh/mavs_rp3d_veh.cpp
	rp3d_veh/mavs_tire.cpp
	rp3d_veh/mavs_rp3d_pacejka.cpp
	rp3d_veh/radial_spring_tire.cpp
	uav/uav.cpp
	uav/uav_sim.cpp
	uav/load_uav_sim.cpp
	uav/uav_controller.cpp
	helicopter/mavs_helicopter.cpp
	chrono/chrono_wheeled_json.cpp
)
else (MAVS_USE_CHRONO)
add_library(mavs_vehicle STATIC
	autonomous_vehicle.cpp
	#turtle/turtle.cpp 
	#bicycle/bicycle.cpp 
	#car/full_car.cpp
	controllers/pid_controller.cpp
	controllers/pure_pursuit_controller.cpp
	rp3d_veh/mavs_rp3d_veh.cpp
	rp3d_veh/mavs_tire.cpp
	rp3d_veh/mavs_rp3d_pacejka.cpp
	rp3d_veh/radial_spring_tire.cpp
	uav/uav.cpp
	uav/uav_sim.cpp
	uav/load_uav_sim.cpp
	uav/uav_controller.cpp
	helicopter/mavs_helicopter.cpp
)
endif(MAVS_USE_CHRONO)

target_include_directories(mavs_vehicle
PUBLIC # USE (INTERFACE) and BUILD (PRIVATE) headers
	$<INSTALL_INTERFACE:include> # Uses this when installed
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Uses this when building
PRIVATE # BUILD-only includes
	${CMAKE_CURRENT_SOURCE_DIR}
)

if (MAVS_USE_CHRONO)
find_package(Chrono
             COMPONENTS Vehicle
             CONFIG)

	#target_sources(mavs_vehicle chrono/chrono_wheeled_json.cpp)
	# We have to separate the Chrono includes to allow them to be captured here
	target_include_directories(mavs_vehicle
	PUBLIC # USE (INTERFACE) and BUILD (PRIVATE) headers
		$<INSTALL_INTERFACE:include> # Uses this when installed
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/chrono/include> # Uses this when building
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vti/include>
		${CHRONO_INCLUDE_DIRS}
	)
	#--------------------------------------------------------------
	# Tweaks to disable some warnings with MSVC
	#--------------------------------------------------------------
	# TODO: Cleanup and re/move
	if(MSVC)
		add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
		add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
		add_definitions( "-DNOMINMAX" )                # do not use MSVC's min/max macros
		set(EXTRA_COMPILE_FLAGS "/wd4275")             # disable warnings triggered by Irrlicht
	endif()
	
	if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	  if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
		add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
	  endif()
	endif()
endif()

# Build the object library position independent
set_target_properties(mavs_vehicle PROPERTIES POSITION_INDEPENDENT_CODE True)

target_link_libraries(mavs_vehicle
PUBLIC
	mavs_core
	mavs_interfaces
	$<BUILD_INTERFACE:ReactPhysics3D::reactphysics3d> # This may need some aliasing
	$<INSTALL_INTERFACE:ReactPhysics3D::ReactPhysics3D>
	${CHRONO_LIBRARIES}
PRIVATE
	mavs_vti
	glm::glm
	rapidjson
)


#####################################
# Target install
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if (MAVS_USE_CHRONO)
	install(DIRECTORY chrono/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()
install(TARGETS mavs_vehicle
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)