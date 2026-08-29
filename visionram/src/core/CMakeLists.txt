add_library(visionarm_contract STATIC
    ${PROJECT_SOURCE_DIR}/src/common/monotonic_clock.cpp)
add_library(visionarm::contract ALIAS visionarm_contract)
target_include_directories(visionarm_contract PUBLIC ${PROJECT_SOURCE_DIR}/include)
visionarm_apply_warnings(visionarm_contract)
