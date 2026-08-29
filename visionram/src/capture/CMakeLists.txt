add_library(visionarm_capture STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/capture_buffer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dmabuf.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/v4l2_camera.cpp)
add_library(visionarm::capture ALIAS visionarm_capture)
target_link_libraries(visionarm_capture PUBLIC visionarm::contract Threads::Threads)
visionarm_apply_warnings(visionarm_capture)
