function(make_mavs_util name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE mavs_sim)
    install(TARGETS ${name} RUNTIME DESTINATION bin/utilities)
endfunction()

set(MAVS_UTILS
    mesh_manipulator
    #create_ecosystem
    #dem_loader
	fix_texture
)

if (MAVS_USE_EMBREE)
    set(MAVS_UTILS ${MAVS_UTILS} 
        top_down_ortho
        obj_viewer
        rp3d_vehicle_viewer
        scene_viewer
		mavs_benchmark
        heightmap_generator
		#heightmap_to_mesh
        path_tracer_render
        batch_obj_convert)
    add_subdirectory(scene_editor)
endif()

foreach(util IN LISTS MAVS_UTILS)
    make_mavs_util(${util})
endforeach()

