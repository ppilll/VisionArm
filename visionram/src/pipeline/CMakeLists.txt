add_library(visionarm_pipeline STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inference_pipeline.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/latency_metrics.cpp)
add_library(visionarm::pipeline ALIAS visionarm_pipeline)
target_link_libraries(visionarm_pipeline
    PUBLIC
        visionarm::capture
        visionarm::perception
        visionarm::video
        visionarm::logging
        Threads::Threads)
visionarm_apply_warnings(visionarm_pipeline)
