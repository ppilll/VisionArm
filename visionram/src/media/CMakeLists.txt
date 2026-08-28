add_library(visionarm_media STATIC ${PROJECT_SOURCE_DIR}/src/media/media_clock.cpp)
add_library(visionarm::media ALIAS visionarm_media)
target_link_libraries(visionarm_media PUBLIC visionarm::contract Threads::Threads)
visionarm_apply_warnings(visionarm_media)

if(VISIONARM_ENABLE_FFMPEG_MP4_MUX)
    add_library(visionarm_media_mux STATIC ${PROJECT_SOURCE_DIR}/src/media/mp4_recorder.cpp)
    add_library(visionarm::media_mux ALIAS visionarm_media_mux)
    target_include_directories(visionarm_media_mux SYSTEM PRIVATE ${FFMPEG_INCLUDE_DIR})
    target_link_libraries(visionarm_media_mux
        PUBLIC visionarm::contract visionarm::logging
        PRIVATE ${FFMPEG_AVFORMAT_LIBRARY} ${FFMPEG_AVCODEC_LIBRARY} ${FFMPEG_AVUTIL_LIBRARY})
    visionarm_apply_warnings(visionarm_media_mux)
endif()

if(VISIONARM_ENABLE_FFMPEG_MPEGTS_NETWORK)
    add_library(visionarm_media_network STATIC ${PROJECT_SOURCE_DIR}/src/media/network_stream.cpp)
    add_library(visionarm::media_network ALIAS visionarm_media_network)
    target_include_directories(visionarm_media_network SYSTEM PRIVATE ${FFMPEG_INCLUDE_DIR})
    target_link_libraries(visionarm_media_network
        PUBLIC visionarm::contract Threads::Threads
        PRIVATE ${FFMPEG_AVFORMAT_LIBRARY} ${FFMPEG_AVCODEC_LIBRARY} ${FFMPEG_AVUTIL_LIBRARY})
    visionarm_apply_warnings(visionarm_media_network)
endif()
