if(VISIONARM_BUILD_RUNTIME)
    set(VISIONARM_RUNTIME_SOURCES
        ${PROJECT_SOURCE_DIR}/src/memory/dma_heap_buffer.cpp
        ${PROJECT_SOURCE_DIR}/src/metrics/latency_accumulator.cpp
        ${PROJECT_SOURCE_DIR}/src/inference/rknn_engine.cpp
        ${PROJECT_SOURCE_DIR}/src/postprocess/yolov8_top1_postprocessor.cpp
        ${PROJECT_SOURCE_DIR}/src/pipeline/inference_pipeline.cpp
        ${PROJECT_SOURCE_DIR}/src/preprocess/letterbox_geometry.cpp
        ${PROJECT_SOURCE_DIR}/src/observability/telemetry.cpp
        ${PROJECT_SOURCE_DIR}/src/video/h265_file_sink.cpp
        ${PROJECT_SOURCE_DIR}/src/video/nv12_mpp_layout.cpp)
    if(VISIONARM_ENABLE_OPENCV_PREPROCESS)
        list(APPEND VISIONARM_RUNTIME_SOURCES
            ${PROJECT_SOURCE_DIR}/src/preprocess/nv12_letterbox_preprocessor.cpp)
    endif()
    if(VISIONARM_ENABLE_RGA_PREPROCESS)
        list(APPEND VISIONARM_RUNTIME_SOURCES
            ${PROJECT_SOURCE_DIR}/src/preprocess/rga_letterbox_preprocessor.cpp)
    endif()
    if(VISIONARM_ENABLE_MPP_VIDEO)
        list(APPEND VISIONARM_RUNTIME_SOURCES
            ${PROJECT_SOURCE_DIR}/src/video/mpp_h265_encoder.cpp)
    endif()

    add_library(visionarm_pipeline STATIC ${VISIONARM_RUNTIME_SOURCES})
    add_library(visionarm::pipeline ALIAS visionarm_pipeline)
    target_link_libraries(visionarm_pipeline
        PUBLIC visionarm::capture visionarm::control visionarm::logging ${RKNN_LIBRARY})
    target_include_directories(visionarm_pipeline SYSTEM PUBLIC ${RKNN_INCLUDE_DIR})
    if(VISIONARM_ENABLE_OPENCV_PREPROCESS)
        target_link_libraries(visionarm_pipeline PUBLIC ${OpenCV_LIBS})
        target_compile_definitions(visionarm_pipeline PUBLIC VISIONARM_HAS_OPENCV_PREPROCESS=1)
    endif()
    if(VISIONARM_ENABLE_MPP_VIDEO)
        target_include_directories(visionarm_pipeline SYSTEM PUBLIC ${MPP_INCLUDE_DIR})
        target_link_libraries(visionarm_pipeline PUBLIC ${MPP_LIBRARY})
        target_compile_definitions(visionarm_pipeline PUBLIC VISIONARM_HAS_MPP_VIDEO=1)
    endif()
    if(VISIONARM_ENABLE_RGA_PREPROCESS)
        target_include_directories(visionarm_pipeline SYSTEM PUBLIC ${RGA_INCLUDE_DIR})
        target_link_libraries(visionarm_pipeline PUBLIC ${RGA_LIBRARY})
        target_compile_definitions(visionarm_pipeline PUBLIC VISIONARM_HAS_RGA_PREPROCESS=1)
    endif()
    visionarm_apply_warnings(visionarm_pipeline)
endif()
