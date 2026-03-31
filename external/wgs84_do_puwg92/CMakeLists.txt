cmake_minimum_required(VERSION 4.0.0)

project(wgs84_do_puwg92)

set(WGS84_DO_PUWG92_INCLUDE_DIRECTORIES ${EXTERNAL_LIBRARIES_DIRECTORY}/wgs84_do_puwg92/include)

add_library(wgs84_do_puwg92 STATIC
    src/wgs84_do_puwg92.cc
    ${WGS84_DO_PUWG92_INCLUDE_DIRECTORIES}/wgs84_do_puwg92/wgs84_do_puwg92.h)

target_include_directories(wgs84_do_puwg92 PUBLIC ${WGS84_DO_PUWG92_INCLUDE_DIRECTORIES})