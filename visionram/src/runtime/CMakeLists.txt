# Registered by the final integration thread after the legacy runtime target is
# removed from cmake/LegacyTargets.cmake.  Keeping the complete target recipe
# next to its sources prevents a second, hand-maintained feature matrix.
add_library(visionarm_runtime_lib STATIC
    runtime_app.cpp
    runtime_cli.cpp
    runtime_report.cpp)
add_library(visionarm::runtime ALIAS visionarm_runtime_lib)

target_link_libraries(visionarm_runtime_lib
    PUBLIC visionarm::pipeline visionarm::logging visionarm::report)

if(VISIONARM_ENABLE_UART_CONTROL)
    target_link_libraries(visionarm_runtime_lib PUBLIC visionarm::uart_adapter)
    target_compile_definitions(visionarm_runtime_lib
        PUBLIC VISIONARM_HAS_UART_CONTROL=1)
endif()
if(VISIONARM_ENABLE_ALSA_AUDIO)
    target_link_libraries(visionarm_runtime_lib
        PUBLIC visionarm::audio_capture visionarm::media)
    target_compile_definitions(visionarm_runtime_lib PUBLIC VISIONARM_HAS_ALSA_AUDIO=1)
endif()
if(VISIONARM_ENABLE_ALSA_AUDIO AND VISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER AND
   (VISIONARM_ENABLE_FFMPEG_MP4_MUX OR VISIONARM_ENABLE_FFMPEG_MPEGTS_NETWORK))
    target_link_libraries(visionarm_runtime_lib PUBLIC visionarm::audio_encoder)
    target_compile_definitions(visionarm_runtime_lib PUBLIC VISIONARM_HAS_AUDIO_ENCODE=1)
endif()
if(VISIONARM_ENABLE_ALSA_AUDIO AND VISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER AND
   VISIONARM_ENABLE_FFMPEG_MP4_MUX)
    target_link_libraries(visionarm_runtime_lib PUBLIC visionarm::media_mux)
    target_compile_definitions(visionarm_runtime_lib PUBLIC VISIONARM_HAS_AV_MUX=1)
endif()
if(VISIONARM_ENABLE_ALSA_AUDIO AND VISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER AND
   VISIONARM_ENABLE_FFMPEG_MPEGTS_NETWORK)
    target_link_libraries(visionarm_runtime_lib PUBLIC visionarm::media_network)
    target_compile_definitions(visionarm_runtime_lib PUBLIC VISIONARM_HAS_NETWORK_MUX=1)
endif()
visionarm_apply_warnings(visionarm_runtime_lib)

add_executable(visionarm_runtime ${PROJECT_SOURCE_DIR}/app/visionarm_runtime_main.cpp)
target_link_libraries(visionarm_runtime PRIVATE visionarm::runtime)
visionarm_apply_warnings(visionarm_runtime)
