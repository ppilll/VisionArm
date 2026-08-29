cmake_minimum_required(VERSION 3.16)

project(visionarm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

include(cmake/Options.cmake)
include(cmake/Dependencies.cmake)

function(visionarm_apply_warnings target)
    target_compile_options(${target} PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wshadow)
endfunction()

add_subdirectory(src/core)
add_subdirectory(src/capture)
add_subdirectory(src/control)
add_subdirectory(src/observability)
add_subdirectory(src/perception)
add_subdirectory(src/video)
add_subdirectory(src/media)
add_subdirectory(src/audio)
add_subdirectory(src/pipeline)
add_subdirectory(src/runtime)
