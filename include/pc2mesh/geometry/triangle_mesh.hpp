#pragma once
#include <Eigen/Dense>
#include <pc2mesh/geometry/point_cloud.hpp>


namespace pc2mesh {
    namespace geometry {
        struct TriangleMesh {
            TriangleMesh(PointCloud & pcd) : pcd(pcd) {};
            std::vector<Eigen::Vector3i> indices;
            std::vector<Eigen::Vector3d> triangle_normals;
            PointCloud& pcd;
        };
    }  // namespace geometry
}  // namespace open3d