cmake_minimum_required(VERSION 4.0.0)

project(multi_view_tls_registration_step_2)

# Source files
set(SOURCES
    multi_view_tls_registration.cpp perform_experiment.cpp 
    multi_view_tls_registration_gui.cpp multi_view_tls_registration.h
    ../lidar_odometry_step_1/lidar_odometry_utils.cpp
    "../../core/src/utils.cpp"
)

# Windows: add resource file
if(WIN32)
    list(APPEND SOURCES "resource.rc")
endif()

add_executable(
  multi_view_tls_registration_step_2 ${SOURCES}
  )

target_compile_definitions(multi_view_tls_registration_step_2 PRIVATE -DWITH_GUI=1)

target_include_directories(
          multi_view_tls_registration_step_2
          PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${THIRDPARTY_DIRECTORY}/glm
          ${EIGEN3_INCLUDE_DIR}
          ${THIRDPARTY_DIRECTORY}
          ${THIRDPARTY_DIRECTORY}/imgui
          ${THIRDPARTY_DIRECTORY}/imgui/backends
          ${THIRDPARTY_DIRECTORY}/ImGuizmo
          ${THIRDPARTY_DIRECTORY}/glad/include
          ${FREEGLUT_INCLUDE_DIR}
          ${THIRDPARTY_DIRECTORY}/json/include
          ${THIRDPARTY_DIRECTORY}/portable-file-dialogs-master
          ${LASZIP_INCLUDE_DIR}/LASzip/include
          ${THIRDPARTY_DIRECTORY}/observation_equations/codes
          ${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp
          ${EXTERNAL_LIBRARIES_DIRECTORY}/include)

  target_link_libraries(
          multi_view_tls_registration_step_2
          PRIVATE ${FREEGLUT_LIBRARY}
          unordered_dense::unordered_dense
          spdlog::spdlog
          ${OPENGL_gl_LIBRARY}
          OpenGL::GLU
          ${PLATFORM_LASZIP_LIB}
          ${PLATFORM_MISCELLANEOUS_LIBS}
          ${CORE_LIBRARIES}
          ${GUI_LIBRARIES}
          PROJ::proj)

if (MSVC)
    target_compile_options(multi_view_tls_registration_step_2 PRIVATE /bigobj)
endif()

install (TARGETS multi_view_tls_registration_step_2 DESTINATION bin)