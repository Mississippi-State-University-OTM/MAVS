function(add_mpi_test name num_proc)
	add_executable(${name} ${name}.cpp)
	target_link_libraries(${name} PRIVATE mavs_sim)
	add_test(NAME ${name} COMMAND "/usr/bin/mpirun" -np ${num_proc} "./${name}")
endfunction()

function(add_norm_test name)
	add_executable(${name} ${name}.cpp)
	target_link_libraries(${name} PRIVATE mavs_sim mavs_vti)
	add_test(NAME ${name} COMMAND ${name})
endfunction()

set(UTESTS
	utest_mavs_gps
	utest_mavs_rtk
	utest_mavs_imu
	utest_mavs_camera
	utest_mavs_pathtrace
	utest_mavs_spherical_camera
	utest_mavs_camera_distortion
	utest_mavs_solar_position
	utest_mavs_clouds
	utest_mavs_snow
	utest_mavs_lidar
	utest_mavs_radar
	utest_swiftnav_io
	utest_mavs_vti
)

#if (MAVS_USE_EMBREE)
#	set(UTESTS ${UTESTS} utest_mavs_animation utest_mavs_springtire)
#	add_executable(unit_tests 
#		unit_tests.cpp 
#		utest_mavs_vehicle.cpp
#		catch2/catch.cpp)
#
#	target_link_libraries(unit_tests PUBLIC mavs mavs_sim mavs_vehicle)
#	target_link_libraries(unit_tests PRIVATE)
#	add_test(NAME unit_tests COMMAND unit_tests)	
#endif()

if (MAVS_USE_CHRONO)
	set(UTESTS ${UTESTS} utest_mavs_chrono_vehicle)
endif()

foreach(utest IN LISTS UTESTS)
	
	if(USE_MPI_T)
		add_mpi_test(${utest} 8)
	else()
		add_norm_test(${utest})
	endif()
endforeach()

