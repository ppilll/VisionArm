cmake_minimum_required(VERSION 3.13)

project(
    visionarm_v2_camera
    VERSION 1.0.0
    LANGUAGES CXX
)

add_executable(
    v4l2_capture
    main.cpp
    v4l2_capture.cpp
)

target_compile_features(
    v4l2_capture
    PRIVATE
    cxx_std_17
)

target_compile_options(
    v4l2_capture
    PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU"
   AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
    target_link_libraries(
        v4l2_capture
        PRIVATE
        stdc++fs
    )
endif()

set_target_properties(
    v4l2_capture
    PROPERTIES
    CXX_EXTENSIONS OFF
)