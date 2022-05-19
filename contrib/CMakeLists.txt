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

option(PC2MESH_ENABLE_AMD_OFFLOAD OFF)
option(PC2MESH_ENABLE_FULL_OPTIMIZATION OFF)

file(GLOB PC2MESH_OFFLOAD_SRC_FILES offload/src/*.cpp)
add_library(pc2mesh-offload SHARED ${PC2MESH_OFFLOAD_SRC_FILES})
target_include_directories(pc2mesh-offload PUBLIC offload/include)
target_compile_options(pc2mesh-offload PRIVATE -Ofast -ffast-math -fopenmp -mavx2 -mfma -msse4 -msse3)

if (PC2MESH_ENABLE_AMD_OFFLOAD)
    target_compile_options(pc2mesh-offload PRIVATE -fopenmp-targets=amdgcn -fopenmp=libomp -stdlib=libstdc++)
    target_link_options(pc2mesh-offload PRIVATE -fopenmp-targets=amdgcn -fopenmp=libomp -lm -stdlib=libstdc++)
else()
    target_compile_options(pc2mesh-offload PRIVATE -march=x86-64-v3)
endif()


target_link_libraries(pc2mesh-contrib PUBLIC pc2mesh-offload)