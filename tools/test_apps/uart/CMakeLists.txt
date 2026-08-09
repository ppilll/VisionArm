cmake_minimum_required(VERSION 3.16)
project(visionarm_uart LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

function(visionarm_strict_warnings target_name)
    target_compile_options(${target_name} PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endfunction()

add_library(visionarm_uart_protocol STATIC
    src/uart/protocol_frame.cpp
    src/uart/protocol_messages.cpp
    src/uart/protocol_reliable.cpp)
add_library(visionarm::uart_protocol ALIAS visionarm_uart_protocol)
target_include_directories(visionarm_uart_protocol PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
visionarm_strict_warnings(visionarm_uart_protocol)

add_library(visionarm_uart_transport STATIC
    src/uart/serial_port.cpp)
add_library(visionarm::uart_transport ALIAS visionarm_uart_transport)
target_include_directories(visionarm_uart_transport PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
visionarm_strict_warnings(visionarm_uart_transport)

add_library(visionarm_uart_link STATIC
    src/uart/uart_link.cpp)
add_library(visionarm::uart_link ALIAS visionarm_uart_link)
target_include_directories(visionarm_uart_link PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(visionarm_uart_link PUBLIC
    visionarm::uart_protocol
    visionarm::uart_transport
    Threads::Threads)
visionarm_strict_warnings(visionarm_uart_link)

add_executable(visionarm_uart_standalone
    apps/visionarm_uart_standalone/main.cpp)
target_link_libraries(visionarm_uart_standalone PRIVATE
    visionarm::uart_link)
visionarm_strict_warnings(visionarm_uart_standalone)

# Real-board-only protocol qualification utility. This is not a host/PTY/MCU
# simulator: it opens the same physical Linux tty and talks to the real MCU.
add_executable(visionarm_uart_wire_qualification
    apps/visionarm_uart_wire_qualification/main.cpp)
target_link_libraries(visionarm_uart_wire_qualification PRIVATE
    visionarm::uart_protocol
    visionarm::uart_transport)
visionarm_strict_warnings(visionarm_uart_wire_qualification)
