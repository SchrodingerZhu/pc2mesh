#pragma once

#include <vector>
#include <pc2mesh/geometry/triangle_mesh.hpp>
#include <utility>
namespace pc2mesh::geometry {
    TriangleMesh create_triangle_mesh_possion(
            PointCloud &pcd,
            size_t depth = 8,
            float width = 0.0f,
            float scale = 1.1f,
            bool linear_fit = false,
            int n_threads = -1);

}  // namespace open3d