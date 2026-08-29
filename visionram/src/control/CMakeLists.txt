add_library(visionarm_control STATIC
    ${PROJECT_SOURCE_DIR}/src/control/mock_control_sink.cpp
    ${PROJECT_SOURCE_DIR}/src/control/target_state_machine.cpp)
add_library(visionarm::control ALIAS visionarm_control)
target_include_directories(visionarm_control PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(visionarm_control
    PUBLIC visionarm::contract Threads::Threads)

if(VISIONARM_ENABLE_UART_CONTROL)
    target_sources(visionarm_control PRIVATE
        ${PROJECT_SOURCE_DIR}/src/control/uart_control_sink.cpp
        ${PROJECT_SOURCE_DIR}/src/control/uart_protocol.cpp
        ${PROJECT_SOURCE_DIR}/src/control/uart_transport.cpp
        ${PROJECT_SOURCE_DIR}/src/control/uart_link.cpp)
endif()
visionarm_apply_warnings(visionarm_control)
