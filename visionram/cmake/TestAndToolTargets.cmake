# Regression tests and optional diagnostic tools remain separately selectable
# from the formal board runtime target.
if(VISIONARM_BUILD_CAPTURE_TOOLS)
    add_executable(v4l2_dmabuf_probe ${PROJECT_SOURCE_DIR}/tools/v4l2_dmabuf_probe.cpp)
    target_link_libraries(v4l2_dmabuf_probe PRIVATE visionarm::capture)
    visionarm_apply_warnings(v4l2_dmabuf_probe)
endif()

if(VISIONARM_BUILD_RUNTIME AND VISIONARM_BUILD_ACCELERATION_TOOLS)
    add_executable(rknn_io_benchmark ${PROJECT_SOURCE_DIR}/tools/rknn_io_benchmark.cpp)
    target_link_libraries(rknn_io_benchmark PRIVATE visionarm::pipeline)
    visionarm_apply_warnings(rknn_io_benchmark)

    if(VISIONARM_ENABLE_RGA_PREPROCESS AND VISIONARM_ENABLE_OPENCV_PREPROCESS)
        add_executable(rga_preprocess_probe ${PROJECT_SOURCE_DIR}/tools/rga_preprocess_probe.cpp)
        target_link_libraries(rga_preprocess_probe PRIVATE visionarm::pipeline)
        visionarm_apply_warnings(rga_preprocess_probe)
    endif()

endif()

if(VISIONARM_BUILD_TESTS)
    enable_testing()

    add_executable(logger_test ${PROJECT_SOURCE_DIR}/tests/logger_test.cpp)
    target_link_libraries(logger_test PRIVATE visionarm::logging)
    visionarm_apply_warnings(logger_test)
    add_executable(runtime_report_test ${PROJECT_SOURCE_DIR}/tests/runtime_report_test.cpp)
    target_link_libraries(runtime_report_test PRIVATE visionarm::report)
    visionarm_apply_warnings(runtime_report_test)

    add_executable(capture_buffer_contract_test ${PROJECT_SOURCE_DIR}/tests/capture_buffer_contract_test.cpp)
    target_link_libraries(capture_buffer_contract_test PRIVATE visionarm::contract)
    add_executable(capture_buffer_broker_test ${PROJECT_SOURCE_DIR}/tests/capture_buffer_broker_test.cpp)
    target_link_libraries(capture_buffer_broker_test PRIVATE visionarm::capture)
    add_executable(v4l2_dmabuf_contract_test ${PROJECT_SOURCE_DIR}/tests/v4l2_dmabuf_contract_test.cpp)
    target_link_libraries(v4l2_dmabuf_contract_test PRIVATE visionarm::capture)
    add_executable(letterbox_geometry_test
        ${PROJECT_SOURCE_DIR}/tests/letterbox_geometry_test.cpp
        ${PROJECT_SOURCE_DIR}/src/preprocess/letterbox_geometry.cpp)
    target_link_libraries(letterbox_geometry_test PRIVATE visionarm::contract)
    add_executable(nv12_mpp_layout_test
        ${PROJECT_SOURCE_DIR}/tests/nv12_mpp_layout_test.cpp
        ${PROJECT_SOURCE_DIR}/src/video/nv12_mpp_layout.cpp)
    target_link_libraries(nv12_mpp_layout_test PRIVATE visionarm::contract)
    add_executable(yolov8_top1_postprocessor_test
        ${PROJECT_SOURCE_DIR}/tests/yolov8_top1_postprocessor_test.cpp
        ${PROJECT_SOURCE_DIR}/src/postprocess/yolov8_top1_postprocessor.cpp
        ${PROJECT_SOURCE_DIR}/src/preprocess/letterbox_geometry.cpp)
    target_link_libraries(yolov8_top1_postprocessor_test PRIVATE visionarm::contract)
    add_executable(target_state_machine_test
        ${PROJECT_SOURCE_DIR}/tests/target_state_machine_test.cpp
        ${PROJECT_SOURCE_DIR}/src/pipeline/target_state_machine.cpp
        ${PROJECT_SOURCE_DIR}/src/control/mock_control_sink.cpp
        ${PROJECT_SOURCE_DIR}/src/common/monotonic_clock.cpp)
    target_link_libraries(target_state_machine_test PRIVATE visionarm::contract Threads::Threads)
    add_executable(latency_accumulator_test
        ${PROJECT_SOURCE_DIR}/tests/latency_accumulator_test.cpp
        ${PROJECT_SOURCE_DIR}/src/metrics/latency_accumulator.cpp)
    target_link_libraries(latency_accumulator_test PRIVATE visionarm::contract Threads::Threads)
    add_executable(bounded_queue_metrics_test ${PROJECT_SOURCE_DIR}/tests/bounded_queue_metrics_test.cpp)
    target_link_libraries(bounded_queue_metrics_test PRIVATE visionarm::contract Threads::Threads)
    add_executable(audio_chunk_contract_test ${PROJECT_SOURCE_DIR}/tests/audio_chunk_contract_test.cpp)
    target_link_libraries(audio_chunk_contract_test PRIVATE visionarm::contract)
    add_executable(media_clock_test ${PROJECT_SOURCE_DIR}/tests/media_clock_test.cpp)
    target_link_libraries(media_clock_test PRIVATE visionarm::media)
    add_executable(encoded_audio_packet_contract_test
        ${PROJECT_SOURCE_DIR}/tests/encoded_audio_packet_contract_test.cpp)
    target_link_libraries(encoded_audio_packet_contract_test PRIVATE visionarm::contract)
    add_executable(udp_telemetry_sink_test
        ${PROJECT_SOURCE_DIR}/tests/udp_telemetry_sink_test.cpp
        ${PROJECT_SOURCE_DIR}/src/observability/telemetry.cpp
        ${PROJECT_SOURCE_DIR}/src/common/monotonic_clock.cpp)
    target_link_libraries(udp_telemetry_sink_test PRIVATE visionarm::contract Threads::Threads)

    if(VISIONARM_ENABLE_UART_CONTROL)
        add_executable(uart_control_sink_test ${PROJECT_SOURCE_DIR}/tests/uart_control_sink_test.cpp)
        target_link_libraries(uart_control_sink_test PRIVATE visionarm::uart_adapter)
        visionarm_apply_warnings(uart_control_sink_test)
    endif()

    foreach(test_target IN ITEMS
            capture_buffer_contract_test capture_buffer_broker_test v4l2_dmabuf_contract_test
            letterbox_geometry_test nv12_mpp_layout_test yolov8_top1_postprocessor_test
            target_state_machine_test latency_accumulator_test bounded_queue_metrics_test
            audio_chunk_contract_test media_clock_test encoded_audio_packet_contract_test
            udp_telemetry_sink_test)
        visionarm_apply_warnings(${test_target})
    endforeach()

    foreach(test_target IN ITEMS
            capture_buffer_contract_test capture_buffer_broker_test v4l2_dmabuf_contract_test
            letterbox_geometry_test nv12_mpp_layout_test yolov8_top1_postprocessor_test
            target_state_machine_test latency_accumulator_test bounded_queue_metrics_test
            audio_chunk_contract_test media_clock_test encoded_audio_packet_contract_test
            udp_telemetry_sink_test logger_test runtime_report_test)
        add_test(NAME ${test_target} COMMAND ${test_target})
    endforeach()
    set_tests_properties(
        capture_buffer_contract_test capture_buffer_broker_test v4l2_dmabuf_contract_test
        letterbox_geometry_test nv12_mpp_layout_test yolov8_top1_postprocessor_test
        target_state_machine_test latency_accumulator_test bounded_queue_metrics_test
        audio_chunk_contract_test media_clock_test encoded_audio_packet_contract_test
        udp_telemetry_sink_test logger_test runtime_report_test
        PROPERTIES TIMEOUT 15)

    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        add_test(NAME runtime_report_validation_test COMMAND
            ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/runtime_report_validation_test.py)
        add_test(NAME visionarm_viewer_test COMMAND
            ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/visionarm_viewer_test.py)
        set_tests_properties(runtime_report_validation_test visionarm_viewer_test
            PROPERTIES TIMEOUT 15)
    endif()

    if(VISIONARM_ENABLE_UART_CONTROL)
        add_test(NAME uart_control_sink_test COMMAND uart_control_sink_test)
        set_tests_properties(uart_control_sink_test PROPERTIES TIMEOUT 15)
    endif()
endif()
