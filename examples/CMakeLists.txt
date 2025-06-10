# function for examples using mavs
function(make_mavs_ex name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE mavs_sim)
    install(TARGETS ${name} RUNTIME DESTINATION bin/examples)
endfunction(make_mavs_ex)

add_subdirectory(halo)

set(EXAMPLES
    actor_example
    add_rain_to_existing
    camera_example
	curve_fitter_example
    create_random_surface

    forester_example_simple
    forester_example
    free_camera_example
    hmf_read_write_example
    lidar_example
    
    multi_vehicle_example
    
    pts_read_write_example
    radar_example
    
    simulation_example
    sky_example
    slam_example
    surface_roughness_stats2D
	
	date_time_example
	mavs_data_path_example
    
)
if(MAVS_USE_EMBREE)
    set(EXAMPLES ${EXAMPLES}
        pathtrace_example
		oakd_camera_example
        lidar_roughness_analyzer
        rp3d_driving_example
        uav_example
		helicopter_example
		chamfer_distance_example
        drop_test_example
    )
    if(MAVS_USE_CHRONO)
        set(EXAMPLES ${EXAMPLES} chrono_driving_example)
    endif()
endif()

foreach(ex IN LISTS EXAMPLES)
    make_mavs_ex(${ex})
endforeach()
unset(EXAMPLES)

# additional links
target_link_libraries(create_random_surface PRIVATE TinyFileDialogs::TinyFileDialogs)


# TODO: Figure out if this is still necessary
if(WIN32)
	set_target_properties(simulation_example PROPERTIES COMPILE_DEFINITIONS BUILDER_STATIC_DEFINE)					  
endif()


  

