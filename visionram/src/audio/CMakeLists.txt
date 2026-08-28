if(VISIONARM_ENABLE_FFMPEG_AUDIO_ENCODER)
    add_library(visionarm_audio_encoder STATIC
        ${PROJECT_SOURCE_DIR}/src/audio/audio_encode.cpp)
    add_library(visionarm::audio_encoder ALIAS visionarm_audio_encoder)
    target_include_directories(visionarm_audio_encoder SYSTEM PRIVATE ${FFMPEG_INCLUDE_DIR})
    target_link_libraries(visionarm_audio_encoder
        PUBLIC visionarm::contract visionarm::logging
        PRIVATE ${FFMPEG_AVCODEC_LIBRARY} ${FFMPEG_AVUTIL_LIBRARY})
    visionarm_apply_warnings(visionarm_audio_encoder)

    add_executable(audio_encode_probe ${PROJECT_SOURCE_DIR}/tools/audio_encode_probe.cpp)
    target_link_libraries(audio_encode_probe PRIVATE visionarm::audio_encoder)
    visionarm_apply_warnings(audio_encode_probe)
endif()

if(VISIONARM_ENABLE_ALSA_AUDIO)
    add_library(visionarm_audio_capture STATIC
        ${PROJECT_SOURCE_DIR}/src/audio/audio_capture.cpp)
    add_library(visionarm::audio_capture ALIAS visionarm_audio_capture)
    target_link_libraries(visionarm_audio_capture
        PUBLIC visionarm::contract visionarm::media Threads::Threads)
    if(TARGET PkgConfig::ALSA)
        target_link_libraries(visionarm_audio_capture PUBLIC PkgConfig::ALSA)
    else()
        target_include_directories(visionarm_audio_capture SYSTEM PRIVATE ${ALSA_INCLUDE_DIR})
        target_link_libraries(visionarm_audio_capture PRIVATE ${ALSA_LIBRARY})
    endif()
    visionarm_apply_warnings(visionarm_audio_capture)
endif()
