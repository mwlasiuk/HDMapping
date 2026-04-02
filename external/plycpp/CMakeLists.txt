cmake_minimum_required(VERSION 4.0.0)

project(plycpp)

set(PLYCPP_INCLUDE_DIRECTORIES ${EXTERNAL_LIBRARIES_DIRECTORY}/plycpp/include)


add_library(plycpp STATIC
    src/plycpp.cpp
    ${PLYCPP_INCLUDE_DIRECTORIES}/plycpp/plycpp.h)

target_include_directories(plycpp PUBLIC ${PLYCPP_INCLUDE_DIRECTORIES})

set_target_properties(plycpp PROPERTIES POSITION_INDEPENDENT_CODE ON)