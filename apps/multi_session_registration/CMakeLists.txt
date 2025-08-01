cmake_minimum_required(VERSION 3.15.0)

project(multi_session_registration_step_3)

add_executable(multi_session_registration_step_3 multi_session_registration.cpp)

target_compile_definitions(multi_session_registration_step_3 PRIVATE -DWITH_GUI=1)

target_include_directories(
  multi_session_registration_step_3
  PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${REPOSITORY_DIRECTORY}/core_hd_mapping/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glm
          ${EIGEN3_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui/backends
          ${EXTERNAL_LIBRARIES_DIRECTORY}/ImGuizmo
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/portable-file-dialogs-master
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glew-2.2.0/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
          ${EXTERNAL_LIBRARIES_DIRECTORY}/freeglut/include)

target_link_libraries(
  multi_session_registration_step_3
  ${FREEGLUT_LIBRARY}
  ${OPENGL_gl_LIBRARY}
  OpenGL::GLU
  ${PLATFORM_LASZIP_LIB}
  ${PLATFORM_MISCELLANEOUS_LIBS}
  ${CORE_LIBRARIES}
  ${GUI_LIBRARIES})

if(WIN32)
  add_custom_command(
    TARGET multi_session_registration_step_3
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E copy
      $<TARGET_RUNTIME_DLLS:multi_session_registration_step_3>
      $<TARGET_FILE_DIR:multi_session_registration_step_3>
    COMMAND_EXPAND_LISTS)
endif()

if (MSVC)
    target_compile_options(multi_session_registration_step_3 PRIVATE /bigobj)
endif()

install (TARGETS multi_session_registration_step_3 DESTINATION bin)