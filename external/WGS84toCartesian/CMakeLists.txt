cmake_minimum_required(VERSION 4.0.0)

project(WGS84toCartesian)

set(WGS84TOCARTESIAN_INCLUDE_DIRECTORIES ${EXTERNAL_LIBRARIES_DIRECTORY}/WGS84toCartesian/include)

add_library(WGS84toCartesian STATIC
    src/WGS84toCartesian.cpp
    ${WGS84TOCARTESIAN_INCLUDE_DIRECTORIES}/WGS84toCartesian/WGS84toCartesian.hpp)

target_include_directories(WGS84toCartesian PUBLIC ${WGS84TOCARTESIAN_INCLUDE_DIRECTORIES})