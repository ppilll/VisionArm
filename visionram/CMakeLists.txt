cmake_minimum_required(VERSION 3.16)
project(visionram_inference_pipeline_dmabuf LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(
    VISIONARM_BUILD_RUNTIME
    "Build the RKNN inference pipeline runtime library"
    ON)
option(
    VISIONARM_BUILD_TESTS
    "Build host-side contract and unit tests"
    ON)
option(
    VISIONARM_BUILD_CAPTURE_TOOLS
    "Build the V4L2 DMA-BUF board probe"
    ON)
option(
    VISIONARM_BUILD_ACCELERATION_TOOLS
    "Build RGA preprocess and RKNN I/O benchmark tools"
    ON)
option(
    VISIONARM_ENABLE_OPENCV_PREPROCESS
    "Build the CPU/OpenCV NV12 letterbox fallback preprocessor"
    ON)
option(
    VISIONARM_ENABLE_RGA_PREPROCESS
    "Build the real librga DMA-BUF preprocessor"
    OFF)
option(
    VISIONARM_ENABLE_MPP_VIDEO
    "Build the Rockchip MPP H.265 VideoLease branch"
    OFF)

find_package(Threads REQUIRED)

add_library(visionarm_contract INTERFACE)
add_library(visionarm::contract ALIAS visionarm_contract)
target_include_directories(
    visionarm_contract
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(
    visionarm_capture
    STATIC
        src/common/monotonic_clock.cpp
        src/camera/capture_buffer_broker.cpp
        src/camera/dmabuf_cpu_sync.cpp
        src/camera/v4l2_dmabuf_contract.cpp
        src/camera/v4l2_camera.cpp)
add_library(visionarm::capture ALIAS visionarm_capture)
target_link_libraries(
    visionarm_capture
    PUBLIC
        visionarm::contract
        Threads::Threads)
target_compile_options(
    visionarm_capture
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wshadow)

if(VISIONARM_BUILD_RUNTIME)
    set(
        RKNN_INCLUDE_DIR
        ""
        CACHE PATH
        "Directory containing rknn_api.h")
    set(
        RKNN_LIBRARY
        ""
        CACHE FILEPATH
        "Full path to librknnrt.so or the platform RKNN runtime library")

    if(NOT RKNN_INCLUDE_DIR)
        message(FATAL_ERROR
            "Set RKNN_INCLUDE_DIR to the directory containing rknn_api.h")
    endif()
    if(NOT RKNN_LIBRARY)
        message(FATAL_ERROR
            "Set RKNN_LIBRARY to the RKNN runtime library")
    endif()

    set(VISIONARM_RUNTIME_SOURCES
        src/memory/dma_heap_buffer.cpp
        src/inference/rknn_engine.cpp
        src/postprocess/yolov8_top1_postprocessor.cpp
        src/pipeline/inference_pipeline.cpp
        src/preprocess/letterbox_geometry.cpp
        src/video/h265_file_sink.cpp
        src/video/nv12_mpp_layout.cpp)

    if(VISIONARM_ENABLE_OPENCV_PREPROCESS)
        find_package(OpenCV REQUIRED COMPONENTS core imgproc)
        list(APPEND VISIONARM_RUNTIME_SOURCES
            src/preprocess/nv12_letterbox_preprocessor.cpp)
    endif()

    if(VISIONARM_ENABLE_RGA_PREPROCESS)
        set(
            RGA_INCLUDE_DIR
            ""
            CACHE PATH
            "Directory containing im2d.hpp")
        set(
            RGA_LIBRARY
            ""
            CACHE FILEPATH
            "Full path to librga.so")
        if(NOT RGA_INCLUDE_DIR)
            message(FATAL_ERROR
                "Set RGA_INCLUDE_DIR to the directory containing im2d.hpp")
        endif()
        if(NOT RGA_LIBRARY)
            message(FATAL_ERROR
                "Set RGA_LIBRARY to librga.so")
        endif()
        list(APPEND VISIONARM_RUNTIME_SOURCES
            src/preprocess/rga_letterbox_preprocessor.cpp)
    endif()


    if(VISIONARM_ENABLE_MPP_VIDEO)
        set(
            MPP_INCLUDE_DIR
            ""
            CACHE PATH
            "Directory containing rk_mpi.h and mpp_buffer.h")
        set(
            MPP_LIBRARY
            ""
            CACHE FILEPATH
            "Full path to librockchip_mpp.so")
        if(NOT MPP_INCLUDE_DIR)
            message(FATAL_ERROR
                "Set MPP_INCLUDE_DIR to the Rockchip MPP include directory")
        endif()
        if(NOT MPP_LIBRARY)
            message(FATAL_ERROR
                "Set MPP_LIBRARY to librockchip_mpp.so")
        endif()
        list(APPEND VISIONARM_RUNTIME_SOURCES
            src/video/mpp_h265_encoder.cpp)
    endif()

    add_library(visionarm_pipeline STATIC ${VISIONARM_RUNTIME_SOURCES})
    add_library(visionarm::pipeline ALIAS visionarm_pipeline)
    target_link_libraries(
        visionarm_pipeline
        PUBLIC
            visionarm::capture
            ${RKNN_LIBRARY})
    target_include_directories(
        visionarm_pipeline
        SYSTEM PUBLIC
            ${RKNN_INCLUDE_DIR})

    if(VISIONARM_ENABLE_OPENCV_PREPROCESS)
        target_link_libraries(visionarm_pipeline PUBLIC ${OpenCV_LIBS})
        target_compile_definitions(
            visionarm_pipeline
            PUBLIC VISIONARM_HAS_OPENCV_PREPROCESS=1)
    endif()

    if(VISIONARM_ENABLE_MPP_VIDEO)
        target_include_directories(
            visionarm_pipeline
            SYSTEM PUBLIC
                ${MPP_INCLUDE_DIR})
        target_link_libraries(
            visionarm_pipeline
            PUBLIC
                ${MPP_LIBRARY})
        target_compile_definitions(
            visionarm_pipeline
            PUBLIC VISIONARM_HAS_MPP_VIDEO=1)
    endif()

    if(VISIONARM_ENABLE_RGA_PREPROCESS)
        target_include_directories(
            visionarm_pipeline
            SYSTEM PUBLIC
                ${RGA_INCLUDE_DIR})
        target_link_libraries(
            visionarm_pipeline
            PUBLIC
                ${RGA_LIBRARY})
        target_compile_definitions(
            visionarm_pipeline
            PUBLIC VISIONARM_HAS_RGA_PREPROCESS=1)
    endif()

    target_compile_options(
        visionarm_pipeline
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow)
endif()

if(VISIONARM_BUILD_CAPTURE_TOOLS)
    add_executable(
        v4l2_dmabuf_probe
        tools/v4l2_dmabuf_probe.cpp)
    target_link_libraries(
        v4l2_dmabuf_probe
        PRIVATE
            visionarm::capture)
    target_compile_options(
        v4l2_dmabuf_probe
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow)
endif()

if(VISIONARM_BUILD_RUNTIME AND VISIONARM_BUILD_ACCELERATION_TOOLS)
    add_executable(
        rknn_io_benchmark
        tools/rknn_io_benchmark.cpp)
    target_link_libraries(
        rknn_io_benchmark
        PRIVATE
            visionarm::pipeline)
    target_compile_options(
        rknn_io_benchmark
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow)

    if(VISIONARM_ENABLE_RGA_PREPROCESS AND
       VISIONARM_ENABLE_OPENCV_PREPROCESS)
        add_executable(
            rga_preprocess_probe
            tools/rga_preprocess_probe.cpp)
        target_link_libraries(
            rga_preprocess_probe
            PRIVATE
                visionarm::pipeline)
        target_compile_options(
            rga_preprocess_probe
            PRIVATE
                -Wall
                -Wextra
                -Wpedantic
                -Wconversion
                -Wshadow)
    endif()

    if(VISIONARM_ENABLE_RGA_PREPROCESS AND
       VISIONARM_ENABLE_MPP_VIDEO)
        add_executable(
            vision_pipeline_r5_r6_probe
            tools/vision_pipeline_r5_r6_probe.cpp)
        target_link_libraries(
            vision_pipeline_r5_r6_probe
            PRIVATE
                visionarm::pipeline)
        target_compile_options(
            vision_pipeline_r5_r6_probe
            PRIVATE
                -Wall
                -Wextra
                -Wpedantic
                -Wconversion
                -Wshadow)
    endif()
endif()

if(VISIONARM_BUILD_TESTS)
    enable_testing()

    add_executable(
        capture_buffer_contract_test
        tests/capture_buffer_contract_test.cpp)
    target_link_libraries(
        capture_buffer_contract_test
        PRIVATE
            visionarm::contract)

    add_executable(
        capture_buffer_broker_test
        tests/capture_buffer_broker_test.cpp)
    target_link_libraries(
        capture_buffer_broker_test
        PRIVATE
            visionarm::capture)

    add_executable(
        v4l2_dmabuf_contract_test
        tests/v4l2_dmabuf_contract_test.cpp)
    target_link_libraries(
        v4l2_dmabuf_contract_test
        PRIVATE
            visionarm::capture)

    add_executable(
        letterbox_geometry_test
        tests/letterbox_geometry_test.cpp
        src/preprocess/letterbox_geometry.cpp)
    target_link_libraries(
        letterbox_geometry_test
        PRIVATE
            visionarm::contract)

    add_executable(
        nv12_mpp_layout_test
        tests/nv12_mpp_layout_test.cpp
        src/video/nv12_mpp_layout.cpp)
    target_link_libraries(
        nv12_mpp_layout_test
        PRIVATE
            visionarm::contract)

    foreach(test_target IN ITEMS
            capture_buffer_contract_test
            capture_buffer_broker_test
            v4l2_dmabuf_contract_test
            letterbox_geometry_test
            nv12_mpp_layout_test)
        target_compile_options(
            ${test_target}
            PRIVATE
                -Wall
                -Wextra
                -Wpedantic
                -Wconversion
                -Wshadow)
    endforeach()

    add_test(
        NAME capture_buffer_contract_test
        COMMAND capture_buffer_contract_test)
    add_test(
        NAME capture_buffer_broker_test
        COMMAND capture_buffer_broker_test)
    add_test(
        NAME v4l2_dmabuf_contract_test
        COMMAND v4l2_dmabuf_contract_test)
    add_test(
        NAME letterbox_geometry_test
        COMMAND letterbox_geometry_test)
    add_test(
        NAME nv12_mpp_layout_test
        COMMAND nv12_mpp_layout_test)

    set_tests_properties(
        capture_buffer_contract_test
        capture_buffer_broker_test
        v4l2_dmabuf_contract_test
        letterbox_geometry_test
        nv12_mpp_layout_test
        PROPERTIES TIMEOUT 15)
endif()
