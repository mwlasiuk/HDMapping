cmake_minimum_required(VERSION 4.0.0)

project(precision_forestry_tools)

add_executable(precision_forestry_tools 
  precision_forestry_tools.cpp 
  surface.cpp
  local_shape_features.cpp)

target_compile_definitions(precision_forestry_tools PRIVATE -DWITH_GUI=1)

target_link_libraries(
  precision_forestry_tools
  PRIVATE ${PLATFORM_LASZIP_LIB}
          ${FREEGLUT_LIBRARY}
          ${OPENGL_gl_LIBRARY}
          OpenGL::GLU
          ${GUI_LIBRARIES}
          ${PLATFORM_MISCELLANEOUS_LIBS}
          ${CORE_LIBRARIES})

target_include_directories(
  precision_forestry_tools
  PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${THIRDPARTY_DIRECTORY}/glm
          ${EIGEN3_INCLUDE_DIR}
          ${THIRDPARTY_DIRECTORY}/imgui
          ${THIRDPARTY_DIRECTORY}/imgui/backends
          ${THIRDPARTY_DIRECTORY}/ImGuizmo
          ${THIRDPARTY_DIRECTORY}/glad/include
          ${FREEGLUT_INCLUDE_DIR}
          ${THIRDPARTY_DIRECTORY}/json/include
          ${THIRDPARTY_DIRECTORY}/portable-file-dialogs-master
          ${LASZIP_INCLUDE_DIR}/LASzip/include)

if (MSVC)
    target_compile_options(precision_forestry_tools PRIVATE /bigobj)
endif()

install (TARGETS precision_forestry_tools DESTINATION bin)