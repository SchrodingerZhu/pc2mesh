set(_PC2MESH_CLI_SOURCE_DIR "${pc2mesh_SOURCE_DIR}/contrib/cli")
set(_PC2MESH_CLI_LIBRARY "${CMAKE_CURRENT_BINARY_DIR}/release/${CMAKE_STATIC_LIBRARY_PREFIX}cli${CMAKE_STATIC_LIBRARY_SUFFIX}")
file(GLOB_RECURSE _PC2MESH_CLI_SRCS "${_PC2MESH_CLI_SOURCE_DIR}/src/*.rs")
file(GLOB_RECURSE _PC2MESH_CLI_HEADERS "${_PC2MESH_CLI_SOURCE_DIR}/include/*.h")
add_custom_command(OUTPUT ${_PC2MESH_CLI_LIBRARY}
        COMMENT "Building cli"
        COMMAND cargo build --release --target-dir ${CMAKE_CURRENT_BINARY_DIR}
        VERBATIM
        WORKING_DIRECTORY ${_PC2MESH_CLI_SOURCE_DIR}
        DEPENDS "${_PC2MESH_CLI_SRCS}"
        "${_PC2MESH_CLI_HEADERS}"
        "${_PC2MESH_CLI_SOURCE_DIR}/Cargo.toml")
add_custom_target(pc2mesh-cli-cargo ALL DEPENDS ${_PC2MESH_CLI_LIBRARY})
add_library(pc2mesh-cli STATIC IMPORTED GLOBAL)
set_target_properties(pc2mesh-cli PROPERTIES IMPORTED_LOCATION ${_PC2MESH_CLI_LIBRARY})
add_dependencies(pc2mesh-cli pc2mesh-cli-cargo)
target_include_directories(pc2mesh-cli INTERFACE ${_PC2MESH_CLI_SOURCE_DIR}/include)