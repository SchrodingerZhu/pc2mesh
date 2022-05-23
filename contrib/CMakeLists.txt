file(GLOB PC2MESH_CONTRIB_SRC_FILES src/*.cpp)
add_library(pc2mesh-contrib STATIC ${PC2MESH_CONTRIB_SRC_FILES})
target_compile_options(pc2mesh-contrib PUBLIC -fopenmp -march=native -Ofast -ffast-math)
target_link_options(pc2mesh-contrib PUBLIC -fopenmp)

add_library(rply STATIC rply/rplyfile.h rply/rply.h rply/rply.c)
target_include_directories(rply PUBLIC rply)
target_link_libraries(pc2mesh-contrib PUBLIC rply)

add_subdirectory(nanoflann)
target_link_libraries(pc2mesh-contrib PUBLIC nanoflann::nanoflann)

find_package (Eigen3 REQUIRED NO_MODULE)
target_link_libraries(pc2mesh-contrib PUBLIC Eigen3::Eigen)

target_include_directories(pc2mesh-contrib PUBLIC include)
add_library(Pc2Mesh::Contrib ALIAS pc2mesh-contrib)

find_package(absl REQUIRED)
target_link_libraries(pc2mesh-contrib PUBLIC absl::flat_hash_set absl::hash)

add_subdirectory(cli)
target_link_libraries(pc2mesh-contrib PUBLIC pc2mesh-cli)
