set(VISIONARM_VIDEO_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/h265_file_sink.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/nv12_mpp_layout.cpp)

if(VISIONARM_ENABLE_MPP_VIDEO)
    list(APPEND VISIONARM_VIDEO_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/mpp_h265_encoder.cpp)
endif()

add_library(visionarm_video STATIC ${VISIONARM_VIDEO_SOURCES})
add_library(visionarm::video ALIAS visionarm_video)
target_link_libraries(visionarm_video
    PUBLIC visionarm::contract visionarm::logging)

if(VISIONARM_ENABLE_MPP_VIDEO)
    target_include_directories(visionarm_video SYSTEM PUBLIC ${MPP_INCLUDE_DIR})
    target_link_libraries(visionarm_video PUBLIC ${MPP_LIBRARY})
    target_compile_definitions(visionarm_video
        PUBLIC VISIONARM_HAS_MPP_VIDEO=1)
endif()

visionarm_apply_warnings(visionarm_video)
