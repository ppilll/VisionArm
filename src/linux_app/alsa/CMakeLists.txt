cmake_minimum_required(VERSION 3.16)

project(v2a_audio_capture LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(v2a_audio_capture
    main.cpp
    alsa_capture.cpp
    wav_writer.cpp
)

target_compile_options(v2a_audio_capture PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

find_package(PkgConfig QUIET)

if(PkgConfig_FOUND)
    pkg_check_modules(ALSA QUIET IMPORTED_TARGET alsa)
endif()

if(TARGET PkgConfig::ALSA)
    target_link_libraries(v2a_audio_capture PRIVATE
        PkgConfig::ALSA
    )
else()
    find_path(ALSA_INCLUDE_DIR
        NAMES alsa/asoundlib.h
    )

    find_library(ALSA_LIBRARY
        NAMES asound
    )

    if(NOT ALSA_INCLUDE_DIR OR NOT ALSA_LIBRARY)
        message(FATAL_ERROR
            "alsa-lib development files were not found. "
            "Do not install anything automatically; inspect the BSP/toolchain."
        )
    endif()

    target_include_directories(v2a_audio_capture PRIVATE
        "${ALSA_INCLUDE_DIR}"
    )

    target_link_libraries(v2a_audio_capture PRIVATE
        "${ALSA_LIBRARY}"
    )
endif()