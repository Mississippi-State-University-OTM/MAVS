# MPI
if (MAVS_USE_MPI)
    find_package(MPI REQUIRED)
    # TODO: Tack this definition onto the imported MPI target instead of a global define
    #add_definitions(-DUSE_MPI)
endif()
# OpenMP 
if(MAVS_USE_OMP)
    find_package(OpenMP REQUIRED)
    # TODO: Tack this definition onto the imported OMP target instead of a global define
    #add_definitions(-DUSE_OMP)
endif()

# Chrono
if(MAVS_USE_CHRONO)
    find_package(Chrono
        COMPONENTS Vehicle
        CONFIG
    )
    # TODO: Tack this definition onto the imported Chrono target instead of a global define
    add_definitions(-DUSE_CHRONO)
endif()

# X11
if(NOT WIN32)
    find_package(X11 REQUIRED)
    # This blob is ripped from newer cmake findX11 modules
    # It is necessary for more dated systems and should be removed
    # when support for those is no longer needed
    if (NOT TARGET X11::X11)
        add_library(X11::X11 UNKNOWN IMPORTED)
        set_target_properties(X11::X11 PROPERTIES
            IMPORTED_LOCATION "${X11_X11_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${X11_X11_INCLUDE_PATH}"
        )
    endif()
    # libjpeg
    find_package(JPEG REQUIRED)
    if(NOT TARGET JPEG::JPEG)
        add_library(JPEG::JPEG UNKNOWN IMPORTED)
        if(JPEG_INCLUDE_DIR)
            set_target_properties(JPEG::JPEG PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${JPEG_INCLUDE_DIR}")
        endif()
        if(EXISTS "${JPEG_LIBRARY}")
            set_target_properties(JPEG::JPEG PROPERTIES
                IMPORTED_LINK_INTERFACE_LANGUAGES "C"
                IMPORTED_LOCATION "${JPEG_LIBRARY}")
        endif()
    endif()

endif()
