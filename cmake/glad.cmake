include_guard()

set(GLAD_LIBRARY_DIRECTORY ${THIRDPARTY_DIRECTORY}/glad)

set(GLAD_SOURCE_FILES
    ${GLAD_LIBRARY_DIRECTORY}/src/glad.c)

set(GLAD_HEADER_FILES
    ${GLAD_LIBRARY_DIRECTORY}/include/glad/glad.h
    ${GLAD_LIBRARY_DIRECTORY}/include/KHR/khrplatform.h)

set(GLAD_FILES ${GLAD_SOURCE_FILES} ${GLAD_HEADER_FILES})
add_library(glad STATIC ${GLAD_FILES})
target_include_directories(
    glad PUBLIC ${GLAD_LIBRARY_DIRECTORY}/include)
