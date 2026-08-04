cmake_minimum_required(VERSION 3.16)
project(postprocess LANGUAGES CXX)

option(VISIONARM_WITH_RKNN "Build the RKNN output adapter" OFF)
set(RKNN_INCLUDE_DIR "" CACHE PATH "Directory containing rknn_api.h")
set(RKNN_LIBRARY "" CACHE FILEPATH "Path to librknnrt.so")

add_executable(postprocess
    postprocess.cpp
)

target_compile_features(postprocess PRIVATE cxx_std_17)
target_compile_options(postprocess PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wsign-conversion
)

if(VISIONARM_WITH_RKNN)
    if(NOT RKNN_INCLUDE_DIR)
        message(FATAL_ERROR "Set RKNN_INCLUDE_DIR to the directory containing rknn_api.h")
    endif()

    target_compile_definitions(postprocess PRIVATE
        VISIONARM_WITH_RKNN=1
    )
    target_include_directories(postprocess PRIVATE
        "${RKNN_INCLUDE_DIR}"
    )

    if(RKNN_LIBRARY)
        target_link_libraries(postprocess PRIVATE
            "${RKNN_LIBRARY}"
        )
    endif()
endif()