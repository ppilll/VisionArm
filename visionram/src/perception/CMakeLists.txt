set(VISIONARM_PERCEPTION_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/dma_heap_buffer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/preprocess_geometry.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rknn_engine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/yolov8_postprocess.cpp)

if(VISIONARM_ENABLE_OPENCV_PREPROCESS)
    list(APPEND VISIONARM_PERCEPTION_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/cpu_preprocess.cpp)
endif()
if(VISIONARM_ENABLE_RGA_PREPROCESS)
    list(APPEND VISIONARM_PERCEPTION_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/rga_preprocess.cpp)
endif()

add_library(visionarm_perception STATIC ${VISIONARM_PERCEPTION_SOURCES})
add_library(visionarm::perception ALIAS visionarm_perception)
target_link_libraries(visionarm_perception
    PUBLIC visionarm::contract visionarm::logging ${RKNN_LIBRARY})
target_include_directories(visionarm_perception
    SYSTEM PUBLIC ${RKNN_INCLUDE_DIR})

if(VISIONARM_ENABLE_OPENCV_PREPROCESS)
    target_link_libraries(visionarm_perception PUBLIC ${OpenCV_LIBS})
    target_compile_definitions(visionarm_perception
        PUBLIC VISIONARM_HAS_OPENCV_PREPROCESS=1)
endif()
if(VISIONARM_ENABLE_RGA_PREPROCESS)
    target_include_directories(visionarm_perception
        SYSTEM PUBLIC ${RGA_INCLUDE_DIR})
    target_link_libraries(visionarm_perception PUBLIC ${RGA_LIBRARY})
    target_compile_definitions(visionarm_perception
        PUBLIC VISIONARM_HAS_RGA_PREPROCESS=1)
endif()

visionarm_apply_warnings(visionarm_perception)
