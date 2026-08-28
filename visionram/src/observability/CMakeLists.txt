add_library(visionarm_logging STATIC ${PROJECT_SOURCE_DIR}/src/observability/logger.cpp)
add_library(visionarm::logging ALIAS visionarm_logging)
target_link_libraries(visionarm_logging PUBLIC visionarm::contract Threads::Threads)
target_compile_definitions(visionarm_logging PUBLIC
    VISIONARM_COMPILETIME_LOG_LEVEL=${VISIONARM_DEFAULT_LOG_LEVEL_VALUE})
visionarm_apply_warnings(visionarm_logging)

add_library(visionarm_report STATIC ${PROJECT_SOURCE_DIR}/src/observability/runtime_report.cpp)
add_library(visionarm::report ALIAS visionarm_report)
target_link_libraries(visionarm_report PUBLIC visionarm::contract)
visionarm_apply_warnings(visionarm_report)
