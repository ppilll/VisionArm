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

    # Compatibility aliases preserve the scaffold-era target graph for
    # runtime and regression consumers while exposing one control capability.
    add_library(visionarm::uart_adapter ALIAS visionarm_control)
    add_library(visionarm::uart_protocol ALIAS visionarm_control)
    add_library(visionarm::uart_transport ALIAS visionarm_control)
    add_library(visionarm::uart_link ALIAS visionarm_control)
endif()
visionarm_apply_warnings(visionarm_control)

if(VISIONARM_BUILD_UART_BOARD_TESTS)
    add_executable(uart_motor_control_board_test
        ${PROJECT_SOURCE_DIR}/tests/uart_motor_control_board_test.cpp)
    target_link_libraries(uart_motor_control_board_test PRIVATE visionarm::uart_link)
    visionarm_apply_warnings(uart_motor_control_board_test)

    # Step-K is a real-hardware synthetic controller qualification executable.
    # It intentionally links the frozen protocol + SerialPort layers directly
    # so the test can set capture_age_at_tx_ms=0 and exact 20/100 ms wire
    # cadence without changing the production UartLink implementation.
    add_executable(uart_step_k_synthetic_test
        ${PROJECT_SOURCE_DIR}/tests/uart_step_k_synthetic_test.cpp)
    target_link_libraries(uart_step_k_synthetic_test PRIVATE
        visionarm::uart_protocol visionarm::uart_transport)
    visionarm_apply_warnings(uart_step_k_synthetic_test)
endif()
