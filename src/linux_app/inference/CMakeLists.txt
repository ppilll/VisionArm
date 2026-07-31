cmake_minimum_required(VERSION 3.10)
project(rknn_inference LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(RKNN_INCLUDE_DIR "" CACHE PATH "Directory containing rknn_api.h")
set(RKNN_LIBRARY "" CACHE FILEPATH "Full path to librknnrt.so")

if(NOT EXISTS "${RKNN_INCLUDE_DIR}/rknn_api.h")
    message(FATAL_ERROR "Set RKNN_INCLUDE_DIR to the directory containing rknn_api.h")
endif()

if(NOT EXISTS "${RKNN_LIBRARY}")
    message(FATAL_ERROR "Set RKNN_LIBRARY to the full path of librknnrt.so")
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_executable(rknn_inference
    main.cpp
    rknn_engine.cpp
)

target_include_directories(rknn_inference PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${RKNN_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(rknn_inference PRIVATE
    ${RKNN_LIBRARY}
    ${OpenCV_LIBS}
)

target_compile_options(rknn_inference PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)
