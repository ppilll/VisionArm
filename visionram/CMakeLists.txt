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
option(
    VISIONARM_ENABLE_ALSA_AUDIO
    "Build the ALSA PCM producer and V8.3 audio timeline path"
    ON)
option(
    VISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER
    "Build the FFmpeg native AAC-LC encoder and standalone PCM->AAC probe"
    OFF)
option(
    VISIONARM_ENABLE_FFMPEG_MP4_MUX
    "Build the FFmpeg libavformat HEVC+AAC MP4 local recorder"
    OFF)
option(
    VISIONARM_ENABLE_FFMPEG_MPEGTS_NETWORK
    "Build the asynchronous FFmpeg HEVC+AAC MPEG-TS/UDP network sink"
    OFF)
option(
    VISIONARM_ENABLE_UART_CONTROL
    "Build the integrated R6 UART control path"
    ON)
option(
    VISIONARM_BUILD_UART_BOARD_TESTS
    "Build real RK3588-to-MCU UART motor-control board tests"
    OFF)

if(VISIONARM_ENABLE_FFMPEG_MP4_MUX AND NOT VISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER)
    message(FATAL_ERROR
        "VISIONARM_ENABLE_FFMPEG_MP4_MUX requires VISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER=ON")
endif()

if(VISIONARM_ENABLE_FFMPEG_MPEGTS_NETWORK AND
   NOT VISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER)
    message(FATAL_ERROR
        "VISIONARM_ENABLE_FFMPEG_MPEGTS_NETWORK requires VISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER=ON")
endif()

if(VISIONARM_BUILD_UART_BOARD_TESTS AND NOT VISIONARM_ENABLE_UART_CONTROL)
    message(FATAL_ERROR
        "VISIONARM_BUILD_UART_BOARD_TESTS requires VISIONARM_ENABLE_UART_CONTROL=ON")
endif()

find_package(Threads REQUIRED)

add_library(visionarm_contract INTERFACE)
add_library(visionarm::contract ALIAS visionarm_contract)
target_include_directories(
    visionarm_contract
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(
    visionarm_media
    STATIC
        src/media/media_clock.cpp)
add_library(visionarm::media ALIAS visionarm_media)
target_link_libraries(
    visionarm_media
    PUBLIC
        visionarm::contract
        Threads::Threads)
target_compile_options(
    visionarm_media
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wshadow)

if(VISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER)
    set(
        FFMPEG_INCLUDE_DIR
        ""
        CACHE PATH
        "Directory containing libavcodec/avcodec.h and libavutil headers")
    set(
        FFMPEG_AVCODEC_LIBRARY
        ""
        CACHE FILEPATH
        "Full path to libavcodec.so")
    set(
        FFMPEG_AVUTIL_LIBRARY
        ""
        CACHE FILEPATH
        "Full path to libavutil.so")

    if(NOT FFMPEG_INCLUDE_DIR)
        find_path(
            FFMPEG_INCLUDE_DIR
            NAMES libavcodec/avcodec.h)
    endif()
    if(NOT FFMPEG_AVCODEC_LIBRARY)
        find_library(
            FFMPEG_AVCODEC_LIBRARY
            NAMES avcodec)
    endif()
    if(NOT FFMPEG_AVUTIL_LIBRARY)
        find_library(
            FFMPEG_AVUTIL_LIBRARY
            NAMES avutil)
    endif()

    if(NOT FFMPEG_INCLUDE_DIR OR
       NOT FFMPEG_AVCODEC_LIBRARY OR
       NOT FFMPEG_AVUTIL_LIBRARY)
        message(FATAL_ERROR
            "VISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER=ON requires FFmpeg libavcodec/libavutil development files. "
            "Set FFMPEG_INCLUDE_DIR, FFMPEG_AVCODEC_LIBRARY, and FFMPEG_AVUTIL_LIBRARY if CMake cannot find the Buildroot staging files.")
    endif()

    add_library(
        visionarm_audio_encoder
        STATIC
            src/audio/ffmpeg_aac_encoder.cpp
            src/audio/audio_encode_worker.cpp
            src/audio/encoded_audio_sink_worker.cpp)
    add_library(visionarm::audio_encoder ALIAS visionarm_audio_encoder)
    target_include_directories(
        visionarm_audio_encoder
        SYSTEM PRIVATE
            ${FFMPEG_INCLUDE_DIR})
    target_link_libraries(
        visionarm_audio_encoder
        PUBLIC
            visionarm::contract
        PRIVATE
            ${FFMPEG_AVCODEC_LIBRARY}
            ${FFMPEG_AVUTIL_LIBRARY})
    target_compile_options(
        visionarm_audio_encoder
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow)

    add_executable(
        audio_encode_probe
        tools/audio_encode_probe.cpp)
    target_link_libraries(
        audio_encode_probe
        PRIVATE
            visionarm::audio_encoder)
    target_compile_options(
        audio_encode_probe
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow)
endif()

if(VISIONARM_ENABLE_FFMPEG_MP4_MUX OR
   VISIONARM_ENABLE_FFMPEG_MPEGTS_NETWORK)
    set(
        FFMPEG_AVFORMAT_LIBRARY
        ""
        CACHE FILEPATH
        "Full path to libavformat.so")
    if(NOT FFMPEG_AVFORMAT_LIBRARY)
        find_library(
            FFMPEG_AVFORMAT_LIBRARY
            NAMES avformat)
    endif()
    if(NOT FFMPEG_AVFORMAT_LIBRARY)
        message(FATAL_ERROR
            "FFmpeg MP4/MPEG-TS output requires libavformat. "
            "Set FFMPEG_AVFORMAT_LIBRARY to the Buildroot staging libavformat.so")
    endif()
endif()

if(VISIONARM_ENABLE_FFMPEG_MP4_MUX)
    add_library(
        visionarm_media_mux
        STATIC
            src/media/ffmpeg_mp4_muxer.cpp)
    add_library(visionarm::media_mux ALIAS visionarm_media_mux)
    target_include_directories(
        visionarm_media_mux
        SYSTEM PRIVATE
            ${FFMPEG_INCLUDE_DIR})
    target_link_libraries(
        visionarm_media_mux
        PUBLIC
            visionarm::contract
        PRIVATE
            ${FFMPEG_AVFORMAT_LIBRARY}
            ${FFMPEG_AVCODEC_LIBRARY}
            ${FFMPEG_AVUTIL_LIBRARY})
    target_compile_options(
        visionarm_media_mux
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow)
endif()

if(VISIONARM_ENABLE_FFMPEG_MPEGTS_NETWORK)
    add_library(
        visionarm_media_network
        STATIC
            src/media/ffmpeg_mpegts_udp_sink.cpp)
    add_library(visionarm::media_network ALIAS visionarm_media_network)
    target_include_directories(
        visionarm_media_network
        SYSTEM PRIVATE
            ${FFMPEG_INCLUDE_DIR})
    target_link_libraries(
        visionarm_media_network
        PUBLIC
            visionarm::contract
            Threads::Threads
        PRIVATE
            ${FFMPEG_AVFORMAT_LIBRARY}
            ${FFMPEG_AVCODEC_LIBRARY}
            ${FFMPEG_AVUTIL_LIBRARY})
    target_compile_options(
        visionarm_media_network
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow)
endif()

if(VISIONARM_ENABLE_ALSA_AUDIO)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(ALSA QUIET IMPORTED_TARGET alsa)
    endif()

    if(NOT TARGET PkgConfig::ALSA)
        find_path(ALSA_INCLUDE_DIR NAMES alsa/asoundlib.h)
        find_library(ALSA_LIBRARY NAMES asound)
        if(NOT ALSA_INCLUDE_DIR OR NOT ALSA_LIBRARY)
            message(FATAL_ERROR
                "VISIONARM_ENABLE_ALSA_AUDIO=ON but alsa-lib development files were not found")
        endif()
    endif()

    add_library(
        visionarm_audio_capture
        STATIC
            src/audio/alsa_capture.cpp
            src/audio/audio_capture_worker.cpp)
    add_library(visionarm::audio_capture ALIAS visionarm_audio_capture)
    target_link_libraries(
        visionarm_audio_capture
        PUBLIC
            visionarm::contract
            visionarm::media
            Threads::Threads)
    if(TARGET PkgConfig::ALSA)
        target_link_libraries(
            visionarm_audio_capture
            PUBLIC
                PkgConfig::ALSA)
    else()
        target_include_directories(
            visionarm_audio_capture
            SYSTEM PRIVATE
                ${ALSA_INCLUDE_DIR})
        target_link_libraries(
            visionarm_audio_capture
            PRIVATE
                ${ALSA_LIBRARY})
    endif()
    target_compile_options(
        visionarm_audio_capture
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow)
endif()

# R6-L2 physically integrates the frozen R5 UART product sources into the V4
# tree under include/uart and src/uart.  The library boundaries remain
# protocol -> transport -> link -> V4 adapter; no second UART implementation
# is introduced and no external UART sub-project is required.
if(VISIONARM_ENABLE_UART_CONTROL)
    add_library(visionarm_uart_protocol STATIC
        src/uart/protocol_frame.cpp
        src/uart/protocol_messages.cpp
        src/uart/protocol_reliable.cpp)
    add_library(visionarm::uart_protocol ALIAS visionarm_uart_protocol)
    target_include_directories(
        visionarm_uart_protocol
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_compile_options(
        visionarm_uart_protocol
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow)

    add_library(visionarm_uart_transport STATIC
        src/uart/serial_port.cpp)
    add_library(visionarm::uart_transport ALIAS visionarm_uart_transport)
    target_include_directories(
        visionarm_uart_transport
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_compile_options(
        visionarm_uart_transport
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow)

    add_library(visionarm_uart_link STATIC
        src/uart/uart_link.cpp)
    add_library(visionarm::uart_link ALIAS visionarm_uart_link)
    target_include_directories(
        visionarm_uart_link
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(
        visionarm_uart_link
        PUBLIC
            visionarm::uart_protocol
            visionarm::uart_transport
            Threads::Threads)
    target_compile_options(
        visionarm_uart_link
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow)

    add_library(visionarm_uart_adapter STATIC
        src/control/uart_control_sink.cpp)
    add_library(visionarm::uart_adapter ALIAS visionarm_uart_adapter)
    target_link_libraries(
        visionarm_uart_adapter
        PUBLIC
            visionarm::contract
            visionarm::uart_link)
    target_compile_options(
        visionarm_uart_adapter
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow)
endif()

if(VISIONARM_BUILD_UART_BOARD_TESTS)
    add_executable(
        uart_motor_control_board_test
        tests/uart_motor_control_board_test.cpp)
    target_link_libraries(
        uart_motor_control_board_test
        PRIVATE
            visionarm::uart_link)
    target_compile_options(
        uart_motor_control_board_test
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow)

    # Step-K is a real-hardware synthetic controller qualification executable.
    # It intentionally links the frozen protocol + SerialPort layers directly
    # so the test can set capture_age_at_tx_ms=0 and exact 20/100 ms wire
    # cadence without changing the production UartLink implementation.
    add_executable(
        uart_step_k_synthetic_test
        tests/uart_step_k_synthetic_test.cpp)
    target_link_libraries(
        uart_step_k_synthetic_test
        PRIVATE
            visionarm::uart_protocol
            visionarm::uart_transport)
    target_compile_options(
        uart_step_k_synthetic_test
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow)
endif()

add_library(
    visionarm_capture
    STATIC
        src/common/monotonic_clock.cpp
        src/camera/capture_buffer_broker.cpp
        src/camera/dmabuf_cpu_sync.cpp
        src/camera/v4l2_dmabuf_contract.cpp
        src/camera/v4l2_camera.cpp
        src/camera/v4l2_sensor_controller.cpp)
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
        src/metrics/latency_accumulator.cpp
        src/control/mock_control_sink.cpp
        src/inference/rknn_engine.cpp
        src/postprocess/yolov8_top1_postprocessor.cpp
        src/pipeline/inference_pipeline.cpp
        src/pipeline/target_state_machine.cpp
        src/preprocess/letterbox_geometry.cpp
        src/telemetry/udp_telemetry_sink.cpp
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

        add_executable(
            vision_pipeline_r7_r8_probe
            tools/vision_pipeline_r7_r8_probe.cpp)
        target_link_libraries(
            vision_pipeline_r7_r8_probe
            PRIVATE
                visionarm::pipeline)
        if(VISIONARM_ENABLE_UART_CONTROL)
            target_link_libraries(
                vision_pipeline_r7_r8_probe
                PRIVATE
                    visionarm::uart_adapter)
            target_compile_definitions(
                vision_pipeline_r7_r8_probe
                PRIVATE
                    VISIONARM_HAS_UART_CONTROL=1)
        endif()
        if(VISIONARM_ENABLE_ALSA_AUDIO)
            target_link_libraries(
                vision_pipeline_r7_r8_probe
                PRIVATE
                    visionarm::audio_capture
                    visionarm::media)
            target_compile_definitions(
                vision_pipeline_r7_r8_probe
                PRIVATE
                    VISIONARM_HAS_ALSA_AUDIO=1)
        endif()
        if(VISIONARM_ENABLE_ALSA_AUDIO AND
           VISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER AND
           (VISIONARM_ENABLE_FFMPEG_MP4_MUX OR
            VISIONARM_ENABLE_FFMPEG_MPEGTS_NETWORK))
            target_link_libraries(
                vision_pipeline_r7_r8_probe
                PRIVATE
                    visionarm::audio_encoder)
            target_compile_definitions(
                vision_pipeline_r7_r8_probe
                PRIVATE
                    VISIONARM_HAS_AUDIO_ENCODE=1)
        endif()
        if(VISIONARM_ENABLE_ALSA_AUDIO AND
           VISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER AND
           VISIONARM_ENABLE_FFMPEG_MP4_MUX)
            target_link_libraries(
                vision_pipeline_r7_r8_probe
                PRIVATE
                    visionarm::media_mux)
            target_compile_definitions(
                vision_pipeline_r7_r8_probe
                PRIVATE
                    VISIONARM_HAS_AV_MUX=1)
        endif()
        if(VISIONARM_ENABLE_ALSA_AUDIO AND
           VISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER AND
           VISIONARM_ENABLE_FFMPEG_MPEGTS_NETWORK)
            target_link_libraries(
                vision_pipeline_r7_r8_probe
                PRIVATE
                    visionarm::media_network)
            target_compile_definitions(
                vision_pipeline_r7_r8_probe
                PRIVATE
                    VISIONARM_HAS_NETWORK_MUX=1)
        endif()
        target_compile_options(
            vision_pipeline_r7_r8_probe
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

    add_executable(
        yolov8_top1_postprocessor_test
        tests/yolov8_top1_postprocessor_test.cpp
        src/postprocess/yolov8_top1_postprocessor.cpp
        src/preprocess/letterbox_geometry.cpp)
    target_link_libraries(
        yolov8_top1_postprocessor_test
        PRIVATE
            visionarm::contract)


    add_executable(
        target_state_machine_test
        tests/target_state_machine_test.cpp
        src/pipeline/target_state_machine.cpp
        src/control/mock_control_sink.cpp
        src/common/monotonic_clock.cpp)
    target_link_libraries(
        target_state_machine_test
        PRIVATE
            visionarm::contract
            Threads::Threads)

    add_executable(
        latency_accumulator_test
        tests/latency_accumulator_test.cpp
        src/metrics/latency_accumulator.cpp)
    target_link_libraries(
        latency_accumulator_test
        PRIVATE
            visionarm::contract
            Threads::Threads)

    add_executable(
        bounded_queue_metrics_test
        tests/bounded_queue_metrics_test.cpp)
    target_link_libraries(
        bounded_queue_metrics_test
        PRIVATE
            visionarm::contract
            Threads::Threads)

    add_executable(
        audio_chunk_contract_test
        tests/audio_chunk_contract_test.cpp)
    target_link_libraries(
        audio_chunk_contract_test
        PRIVATE
            visionarm::contract)

    add_executable(
        media_clock_test
        tests/media_clock_test.cpp)
    target_link_libraries(
        media_clock_test
        PRIVATE
            visionarm::media)

    add_executable(
        encoded_audio_packet_contract_test
        tests/encoded_audio_packet_contract_test.cpp)
    target_link_libraries(
        encoded_audio_packet_contract_test
        PRIVATE
            visionarm::contract)

    add_executable(
        udp_telemetry_sink_test
        tests/udp_telemetry_sink_test.cpp
        src/telemetry/udp_telemetry_sink.cpp
        src/common/monotonic_clock.cpp)
    target_link_libraries(
        udp_telemetry_sink_test
        PRIVATE
            visionarm::contract
            Threads::Threads)

    if(VISIONARM_ENABLE_UART_CONTROL)
        add_executable(
            uart_control_sink_test
            tests/uart_control_sink_test.cpp)
        target_link_libraries(
            uart_control_sink_test
            PRIVATE
                visionarm::uart_adapter)
        target_compile_options(
            uart_control_sink_test
            PRIVATE
                -Wall
                -Wextra
                -Wpedantic
                -Wconversion
                -Wshadow)
    endif()

    foreach(test_target IN ITEMS
            capture_buffer_contract_test
            capture_buffer_broker_test
            v4l2_dmabuf_contract_test
            letterbox_geometry_test
            nv12_mpp_layout_test
            yolov8_top1_postprocessor_test
            target_state_machine_test
            latency_accumulator_test
            bounded_queue_metrics_test
            audio_chunk_contract_test
            media_clock_test
            encoded_audio_packet_contract_test
            udp_telemetry_sink_test)
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
    add_test(
        NAME yolov8_top1_postprocessor_test
        COMMAND yolov8_top1_postprocessor_test)

    set_tests_properties(
        capture_buffer_contract_test
        capture_buffer_broker_test
        v4l2_dmabuf_contract_test
        letterbox_geometry_test
        nv12_mpp_layout_test
        yolov8_top1_postprocessor_test
        PROPERTIES TIMEOUT 15)

    add_test(
        NAME target_state_machine_test
        COMMAND target_state_machine_test)
    add_test(
        NAME latency_accumulator_test
        COMMAND latency_accumulator_test)
    add_test(
        NAME bounded_queue_metrics_test
        COMMAND bounded_queue_metrics_test)
    add_test(
        NAME audio_chunk_contract_test
        COMMAND audio_chunk_contract_test)
    add_test(
        NAME media_clock_test
        COMMAND media_clock_test)
    add_test(
        NAME encoded_audio_packet_contract_test
        COMMAND encoded_audio_packet_contract_test)
    add_test(
        NAME udp_telemetry_sink_test
        COMMAND udp_telemetry_sink_test)

    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        add_test(
            NAME v8_5_overlay_core_test
            COMMAND
                ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/v8_5_overlay_core_test.py)
        add_test(
            NAME v8_6_integration_report_test
            COMMAND
                ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/v8_6_integration_report_test.py)
        add_test(
            NAME v8_6_lightweight_viewer_test
            COMMAND
                ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/v8_6_lightweight_viewer_test.py)
        set_tests_properties(
            v8_5_overlay_core_test
            v8_6_integration_report_test
            v8_6_lightweight_viewer_test
            PROPERTIES TIMEOUT 15)
    endif()

    set_tests_properties(
        target_state_machine_test
        latency_accumulator_test
        bounded_queue_metrics_test
        audio_chunk_contract_test
        media_clock_test
        encoded_audio_packet_contract_test
        udp_telemetry_sink_test
        PROPERTIES TIMEOUT 15)

    if(VISIONARM_ENABLE_UART_CONTROL)
        add_test(
            NAME uart_control_sink_test
            COMMAND uart_control_sink_test)
        set_tests_properties(
            uart_control_sink_test
            PROPERTIES TIMEOUT 15)
    endif()
endif()
