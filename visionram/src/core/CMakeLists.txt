add_library(visionarm_contract INTERFACE)
add_library(visionarm::contract ALIAS visionarm_contract)
target_include_directories(visionarm_contract INTERFACE ${PROJECT_SOURCE_DIR}/include)
