set(EXAMPLES
batch_halo_simulation
dis_datagen


halo_datagen
halo_lidar_trainer
lidar_rotation_analysis
lidar_trainer_v2
)
if(MAVS_USE_EMBREE)
    set(EXAMPLES ${EXAMPLES} halo_car_interactive halo_car_lidar_analysis)
endif()
foreach(ex IN LISTS EXAMPLES)
    make_mavs_ex(${ex})
endforeach()
unset(EXAMPLES)