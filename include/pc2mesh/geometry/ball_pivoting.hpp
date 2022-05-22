#pragma once

#include <vector>
#include <pc2mesh/geometry/triangle_mesh.hpp>

namespace pc2mesh::geometry {
    TriangleMesh create_triangle_mesh_ball_pivoting(
            PointCloud &pcd, const std::vector<double> &radii);

}  // namespace open3d