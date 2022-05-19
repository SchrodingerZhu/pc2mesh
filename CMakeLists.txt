cmake_minimum_required(VERSION 3.23)
project(pc2mesh)

set(CMAKE_CXX_STANDARD 20)

file(GLOB PC2MESH_SRC_FILES src/*.cpp)
add_executable(pc2mesh ${PC2MESH_SRC_FILES})
add_subdirectory(contrib)

target_link_libraries(pc2mesh PRIVATE Pc2Mesh::Contrib)
target_include_directories(pc2mesh PRIVATE include)
