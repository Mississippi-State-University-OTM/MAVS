# externs with CMake (that we're using)
# embree
# set build flags for Embree automatically to match what we want
set(EMBREE_BACKFACE_CULLING CACHE BOOL OFF)
set(EMBREE_ISPC_SUPPORT CACHE BOOL OFF)
set(EMBREE_STATIC_LIB CACHE BOOL OFF)
set(EMBREE_TASKING_SYSTEM CACHE STRING INTERNAL)
set(EMBREE_TUTORIALS CACHE BOOL OFF)
#add_subdirectory(embree)
# We're breaking the rules using this target, so we need to manually add interfaces
#set_target_properties(embree PROPERTIES
#    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/embree/include)
add_subdirectory(embree)

# glm
add_subdirectory(glm)

# reactphysics3d
add_subdirectory(reactphysics3d)


# Path to the shim header in your tree
set(RP3D_COMPAT_HEADER "${CMAKE_SOURCE_DIR}/extern/rp3d_compat.h")

# Make sure the shim exists
if(NOT EXISTS "${RP3D_COMPAT_HEADER}")
  message(FATAL_ERROR "Missing ${RP3D_COMPAT_HEADER}. Create it and include \"reactphysics3d/decimal.h\".")
endif()

# Let rp3d see the extern folder (optional if using absolute -include path)
target_include_directories(reactphysics3d PRIVATE "${CMAKE_SOURCE_DIR}/extern")

# Force-include for all compilers (absolute path is safest)
target_compile_options(reactphysics3d PRIVATE
  $<$<CXX_COMPILER_ID:MSVC>:/FI"${RP3D_COMPAT_HEADER}">
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-include "${RP3D_COMPAT_HEADER}">
)

target_compile_options(reactphysics3d INTERFACE
  $<$<CXX_COMPILER_ID:MSVC>:/FI"${RP3D_COMPAT_HEADER}">
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-include "${RP3D_COMPAT_HEADER}">
)

# Optional: keep examples/tests off & PIC on if you configure rp3d from your superbuild
set(RP3D_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(RP3D_ENABLE_TESTS   OFF CACHE BOOL "" FORCE)
set_property(TARGET reactphysics3d PROPERTY POSITION_INDEPENDENT_CODE ON)


# wrappers for external libraries that aren't distributed via Git or other usual means

# CImg
add_library(CImg INTERFACE) # Create an interface (header-only) target for CImg
add_library(CImg::CImg ALIAS CImg)
target_include_directories( CImg
INTERFACE
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/CImg>
)
install(FILES CImg/CImg.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if (NOT WIN32)
    target_link_libraries(CImg INTERFACE X11::X11 JPEG::JPEG)
    # gomp pthread m X11 JPEG::JPEG z)
endif()
install(TARGETS CImg
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# cnpy
add_library(cnpy STATIC cnpy/cnpy.cpp cnpy/cnpy.h)
target_include_directories(cnpy
PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cnpy>
)
set_target_properties(cnpy PROPERTIES POSITION_INDEPENDENT_CODE True)
install(FILES cnpy/cnpy.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS cnpy
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# FastNoise
add_library(fastnoise STATIC FastNoise/FastNoise.cpp FastNoise/FastNoise.h)
target_include_directories(fastnoise
PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/FastNoise>
    )
set_target_properties(fastnoise PROPERTIES POSITION_INDEPENDENT_CODE True)
install(FILES FastNoise/FastNoise.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS fastnoise
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# HosekWilkie
add_library(hosekwilkie STATIC
    HosekWilkie/ArHosekSkyModel.cpp
)
target_include_directories(hosekwilkie
PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/HosekWilkie>
)
set_target_properties(hosekwilkie PROPERTIES POSITION_INDEPENDENT_CODE True)
install(
FILES 
    HosekWilkie/ArHosekSkyModel.h
    HosekWilkie/ArHosekSkyModelData_CIEXYZ.h
    HosekWilkie/ArHosekSkyModelData_RGB.h
    HosekWilkie/ArHosekSkyModelData_Spectral.h
DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS hosekwilkie
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# RapidJSON
# RapidJSON's CMakeLists is too much of a mess to consume, so we roll our own stuff
#add_library(rapidjson INTERFACE IMPORTED GLOBAL)
add_library(rapidjson INTERFACE )
set_target_properties(rapidjson PROPERTIES
    INTERFACE_COMPILE_DEFINITIONS RAPIDJSON_HAS_STDSTRING)
#set_target_properties(rapidjson PROPERTIES
#    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/rapidjson/include)
target_include_directories(rapidjson
INTERFACE
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/rapidjson/include>
)
install(DIRECTORY rapidjson/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS rapidjson
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
) 

# rinex
add_library(rinex STATIC rinex/datetime.cpp rinex/datetime.h rinex/rinex.cpp rinex/rinex.h)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rinex/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/)
target_include_directories(rinex
PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/rinex>
)
set_target_properties(rinex PROPERTIES POSITION_INDEPENDENT_CODE True)
install(FILES rinex/rinex.h rinex/datetime.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS rinex
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
) 

# Tiny Object Loader
add_library(tinyobjloader INTERFACE)
target_include_directories(tinyobjloader
INTERFACE
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tiny_obj_loader>
)
install(FILES tiny_obj_loader/tiny_obj_loader.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS tinyobjloader
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# TinyFileDialogs
add_library(TinyFileDialogs STATIC TinyFileDialogs/tinyfiledialogs.c ) # Create an interface (header-only) target for CImg
set_source_files_properties(TinyFileDialogs/tinyfiledialogs.c PROPERTIES LANGUAGE CXX)
target_include_directories(TinyFileDialogs
PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/TinyFileDialogs>
)
set_target_properties(TinyFileDialogs PROPERTIES POSITION_INDEPENDENT_CODE True)
add_library(TinyFileDialogs::TinyFileDialogs ALIAS TinyFileDialogs)
install(FILES TinyFileDialogs/tinyfiledialogs.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS TinyFileDialogs
	EXPORT mavs-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# TinyXML2
add_subdirectory(tinyxml2)
