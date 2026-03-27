cmake_minimum_required(VERSION 4.0.0)

project(hd-mapping)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON) 

set(HDMAPPING_VERSION_MAJOR 0)
set(HDMAPPING_VERSION_MINOR 101)
set(HDMAPPING_VERSION_PATCH 0)

get_filename_component(REPOSITORY_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
set(THIRDPARTY_DIRECTORY ${REPOSITORY_DIRECTORY}/3rdparty)
set(THIRDPARTY_DIRECTORY_BINARY ${REPOSITORY_DIRECTORY}/3rdpartyBinary)
set(EXTERNAL_LIBRARIES_DIRECTORY ${REPOSITORY_DIRECTORY}/external)

set(OpenGL_GL_PREFERENCE GLVND)
set(THREADS_PREFER_PTHREAD_FLAG ON)

add_definitions(-DHDMAPPING_VERSION_MAJOR=${HDMAPPING_VERSION_MAJOR})
add_definitions(-DHDMAPPING_VERSION_MINOR=${HDMAPPING_VERSION_MINOR})
add_definitions(-DHDMAPPING_VERSION_PATCH=${HDMAPPING_VERSION_PATCH})

if (MSVC)
    add_definitions(-DNOMINMAX)
    add_definitions(-DWIN32_LEAN_AND_MEAN)
endif()

option(BUILD_WITH_CLI_TOOLS "Build with command line tools" ON)
option(UTL_PROFILER_DISABLE "Disable UTL profiler (zero overhead when ON)" ON)
option(BUILD_WITH_BUNDLED_FREEGLUT "Build with bundled FreeGlut" ON)
option(BUILD_WITH_BUNDLED_LIBLASZIP "Build with bundled Lib LASZIP" ON)
option(BUILD_WITH_BUNDLED_ONETBB "Build with bundled oneTBB" ON)
option(PYBIND "Enable pybind11 bindings" ON)

if(APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    add_compile_definitions(USE_EXECUTION_PAR_UNSEQ=0)
else()
    add_compile_definitions(USE_EXECUTION_PAR_UNSEQ=1)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

include_directories(shared/include)

find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

add_definitions(-D_USE_MATH_DEFINES)

set(OUTPUT_BIN_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# 1. Set global defaults (Works for most standard projects)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_BIN_DIRECTORY}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_BIN_DIRECTORY}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_BIN_DIRECTORY}")

# 2. Force multi-config generators (VS, Xcode) to use a flat 'bin' directory
# This prevents 'bin/Debug', 'bin/Release', etc.
foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${OUTPUTCONFIG} CONFIG)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIG} "${OUTPUT_BIN_DIRECTORY}")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIG} "${OUTPUT_BIN_DIRECTORY}")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG} "${OUTPUT_BIN_DIRECTORY}")
endforeach()

# 3. Helper to apply properties ONLY to real (non-alias) targets
function(apply_output_directory_settings TARGET_NAME)
    # Final safety check: ensure it's a real target and not an alias
    if(NOT TARGET ${TARGET_NAME})
        return()
    endif()
    get_target_property(IS_ALIAS ${TARGET_NAME} ALIAS)
    if(IS_ALIAS)
        return()
    endif()

    set_target_properties(${TARGET_NAME} PROPERTIES 
        RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_BIN_DIRECTORY}"
        LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_BIN_DIRECTORY}"
        ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_BIN_DIRECTORY}"
    )
    
    foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
        string(TOUPPER ${OUTPUTCONFIG} CONFIG)
        set_target_properties(${TARGET_NAME} PROPERTIES 
            RUNTIME_OUTPUT_DIRECTORY_${CONFIG} "${OUTPUT_BIN_DIRECTORY}"
            LIBRARY_OUTPUT_DIRECTORY_${CONFIG} "${OUTPUT_BIN_DIRECTORY}"
            ARCHIVE_OUTPUT_DIRECTORY_${CONFIG} "${OUTPUT_BIN_DIRECTORY}"
        )
    endforeach()
endfunction()

# 4. Override commands: Intercept and block ALIAS targets from property logic
macro(add_executable TARGET_NAME)
    _add_executable(${TARGET_NAME} ${ARGN})
    # Scan arguments for ALIAS keyword
    set(_ARGS "${ARGN}")
    if(NOT "${_ARGS}" MATCHES "ALIAS")
        apply_output_directory_settings(${TARGET_NAME})
    endif()
endmacro()

macro(add_library TARGET_NAME)
    _add_library(${TARGET_NAME} ${ARGN})
    # Scan arguments for ALIAS or INTERFACE keywords
    # Interface libraries also do not have output directories
    set(_ARGS "${ARGN}")
    if(NOT "${_ARGS}" MATCHES "ALIAS" AND NOT "${_ARGS}" MATCHES "INTERFACE")
        apply_output_directory_settings(${TARGET_NAME})
    endif()
endmacro()

include(cmake/cpu_optimizations.cmake)
include(cmake/platform.cmake)
include(cmake/glad.cmake)
include(cmake/imgui.cmake)
include(cmake/imguizmo.cmake)
include(cmake/implot.cmake)

add_subdirectory(${THIRDPARTY_DIRECTORY}/eigen)
set(EIGEN3_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/eigen)
MESSAGE(STATUS "Using bundled Eigen3 : ${EIGEN3_INCLUDE_DIR}")

if (BUILD_WITH_BUNDLED_FREEGLUT)
    set(FREEGLUT_BUILD_DEMOS
            OFF
            CACHE BOOL "" FORCE)
    add_subdirectory(${THIRDPARTY_DIRECTORY}/freeglut)
    set(FREEGLUT_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/freeglut/include)
    set(FREEGLUT_LIBRARY freeglut)
    # Suppress unused variable warnings from FreeGlut
    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(freeglut PRIVATE -Wno-unused-const-variable)
    endif()
    MESSAGE(STATUS "Using bundled FreeGlut")
else()
    find_package(GLUT REQUIRED)
    set(FREEGLUT_INCLUDE_DIR ${GLUT_INCLUDE_DIR})
    set(FREEGLUT_LIBRARY ${GLUT_glut_LIBRARY})
    MESSAGE(STATUS "FreeGlut include dir: ${FREEGLUT_INCLUDE_DIR}")
endif ()

if (BUILD_WITH_BUNDLED_LIBLASZIP)
    add_subdirectory(${THIRDPARTY_DIRECTORY}/LASzip)
    set(LASZIP_INCLUDE_DIR ${THIRDPARTY_DIRECTORY})
    set(LASZIP_LIBRARY laszip)
    MESSAGE(STATUS "Using bundled LASzip")
    
    if(APPLE)
        target_compile_definitions(laszip PUBLIC off64_t=off_t)
    endif()
else()
    find_package(LASzip REQUIRED)
    set(LASZIP_INCLUDE_DIR ${LASZIP_INCLUDE_DIR})
    set(LASZIP_LIBRARY ${LASZIP_LIBRARY})
    set(PLATFORM_LASZIP_LIB ${LASZIP_LIBRARY})
    MESSAGE(STATUS "LASzip include dir: ${LASZIP_INCLUDE_DIR}, LASzip library: ${LASZIP_LIBRARY}")
endif ()

# Download or Search for OpenCV
include(${THIRDPARTY_DIRECTORY_BINARY}/OpenCV/CMakeLists.txt)
find_package(OpenCV REQUIRED)
MESSAGE(STATUS "OpenCV include dir: ${OpenCV_INCLUDE_DIRS}, OpenCV librarys: ${OpenCV_LIBS}")


# Download or Search for PROJ
include(${THIRDPARTY_DIRECTORY_BINARY}/Proj/CMakeLists.txt)
MESSAGE(STATUS "PROJ include dir: ${PROJ_INCLUDE_DIR}, PROJ library: ${PROJ_LIBRARY}")
MESSAGE(STATUS "PROJ implementation library: ${PROJ_IMPLIB}, PROJ database: ${PROJ_DB}")

if (BUILD_WITH_BUNDLED_ONETBB)
    # Disable oneTBB components we don't need
    set(TBB_TEST OFF CACHE BOOL "" FORCE)
    set(TBB_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(TBB_STRICT OFF CACHE BOOL "" FORCE)
    set(TBBMALLOC_BUILD OFF CACHE BOOL "" FORCE)
    set(TBB_CPF OFF CACHE BOOL "" FORCE)

    add_subdirectory(${THIRDPARTY_DIRECTORY}/oneTBB)
    MESSAGE(STATUS "Using bundled oneTBB from: ${THIRDPARTY_DIRECTORY}/oneTBB")
else()
    # Find TBB from system
    find_package(TBB REQUIRED)
    MESSAGE(STATUS "Found system TBB")
endif()

# Add unordered_dense header-only library
add_subdirectory(${THIRDPARTY_DIRECTORY}/unordered_dense)
MESSAGE(STATUS "Using bundled unordered_dense from: ${THIRDPARTY_DIRECTORY}/unordered_dense")

# Add spdlog logging library
set(SPDLOG_BUILD_PIC ON CACHE BOOL "Build spdlog with position-independent code")
add_subdirectory(${THIRDPARTY_DIRECTORY}/spdlog)
MESSAGE(STATUS "Using bundled spdlog from: ${THIRDPARTY_DIRECTORY}/spdlog")

add_subdirectory(${THIRDPARTY_DIRECTORY}/UTL)
MESSAGE(STATUS "Using bundled UTL from: ${THIRDPARTY_DIRECTORY}/UTL")

if(PYBIND)
    message(STATUS "PYBIND is enabled: fetching pybind.")
    include(FetchContent)
    set(pybind11_SOURCE_DIR "${CMAKE_BINARY_DIR}/3rdparty/pybind11-src")
    set(pybind11_BINARY_DIR "${CMAKE_BINARY_DIR}/3rdparty/pybind11-build")
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11
        GIT_TAG        v2.12.1
        SOURCE_DIR     ${pybind11_SOURCE_DIR}
        BINARY_DIR     ${pybind11_BINARY_DIR}
    )
    FetchContent_GetProperties(pybind11)
    if(NOT pybind11_POPULATED)
        FetchContent_MakeAvailable(pybind11)
    endif()
    add_subdirectory(pybind)
else()
    message(STATUS "PYBIND is disabled.")
endif()


add_subdirectory(${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp)
add_subdirectory(${THIRDPARTY_DIRECTORY}/Fusion/Fusion)

add_subdirectory(core)
add_subdirectory(core_hd_mapping)

set(CORE_LIBRARIES core core-hd-mapping)
set(GUI_LIBRARIES imgui imguizmo implot)

#add_subdirectory(apps/hd_mapper)

if(BUILD_WITH_CLI_TOOLS)
    add_subdirectory(apps/console_tools)
endif()

add_subdirectory(apps/lidar_odometry_step_1)    
add_subdirectory(apps/manual_color)
add_subdirectory(apps/multi_session_registration)
add_subdirectory(apps/multi_view_tls_registration)
add_subdirectory(apps/split_multi_livox)
add_subdirectory(apps/precision_forestry_tools)
add_subdirectory(apps/mandeye_raw_data_viewer)
add_subdirectory(apps/compare_trajectories)
#add_subdirectory(apps/quick_start_demo)
add_subdirectory(apps/mandeye_mission_recorder_calibration)
add_subdirectory(apps/mandeye_single_session_viewer)
add_subdirectory(apps/livox_mid_360_intrinsic_calibration)
add_subdirectory(apps/single_session_manual_coloring)
add_subdirectory(apps/concatenate_multi_livox)

if(WIN32)
    # Install the DLLs and DB into the 'bin' folder of the ZIP/Installer
    install(FILES 
        "${PROJ_ROOT}/bin/proj_9_3.dll" 
        "${PROJ_ROOT}/bin/z.dll" 
        DESTINATION bin
    )
    # Proj often looks for the DB in ../share/proj relative to the DLL
    install(FILES "${PROJ_DB}" DESTINATION share/proj)
endif()

# CPack configuration
set(CPACK_PACKAGE_NAME "hd_mapping")
set(CPACK_PACKAGE_VERSION "${HDMAPPING_VERSION_MAJOR}.${HDMAPPING_VERSION_MINOR}.${HDMAPPING_VERSION_PATCH}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "HD Mapping debian package for Mandeye")
set(CPACK_PACKAGE_CONTACT "januszbedkowski@gmail.com")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Janusz Bedkowski <januszbedkowski@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.28), libstdc++6 (>= 8.3.0), freeglut3-dev, libeigen3-dev, liblaszip-dev")
if(WIN32)
    set(CPACK_GENERATOR "ZIP")
else()
    set(CPACK_GENERATOR "DEB")
endif()

set(CPACK_DEBIAN_PACKAGE_SECTION "utils")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")

include(CPack)