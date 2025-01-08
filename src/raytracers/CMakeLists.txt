#####################################
# Target setup
add_library(mavs_raytracer STATIC
    reflectance_spectrum.cpp
    mesh.cpp 
    bounding_box.cpp
    bounding_cylinder.cpp
    animation.cpp
    fresnel.cpp
    mtl_viewer.cpp
    texture_layers/texture_layer.cpp
    texture_layers/layered_surface.cpp
    simple_tracer/sphere.cpp 
    simple_tracer/aabb.cpp 
    simple_tracer/simple_tracer.cpp
)
target_include_directories(mavs_raytracer
PUBLIC # USE (INTERFACE) and BUILD (PRIVATE) headers
	$<INSTALL_INTERFACE:include> # Uses this when installed
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Uses this when building
)
# Build the object library position independent
set_target_properties(mavs_raytracer PROPERTIES POSITION_INDEPENDENT_CODE True)
target_link_libraries(mavs_raytracer
PUBLIC
    mavs_sensor
    mavs_core
    CImg::CImg
    glm::glm
PRIVATE
    tinyobjloader
)
if (MAVS_USE_EMBREE)
    target_compile_definitions(mavs_raytracer PUBLIC -DUSE_EMBREE)
    target_sources(mavs_raytracer PRIVATE embree_tracer/embree_tracer.cpp )
    target_include_directories(mavs_raytracer
    PUBLIC # USE (INTERFACE) and BUILD (PRIVATE) headers
        $<INSTALL_INTERFACE:include> # Uses this when installed
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/embree_tracer/include> # Uses this when building
    PRIVATE # BUILD-only includes
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_link_libraries(mavs_raytracer
    PUBLIC
        rapidjson
        embree
    PRIVATE
    )
    # MPI is only used with Embree
    if (MAVS_USE_MPI)
        target_link_libraries(mavs_raytracer
        PUBLIC
            MPI::MPI_CXX
        )
    endif()
endif()

#####################################
# Target install
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if (MAVS_USE_EMBREE)
    install(DIRECTORY embree_tracer/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()
install(TARGETS mavs_raytracer
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
  
