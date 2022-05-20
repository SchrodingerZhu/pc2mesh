#pragma once

#include <pc2mesh/geometry/kdtree_flann.hpp>
#include <Eigen/Eigen>
namespace pc2mesh::geometry {
    std::vector<Eigen::Matrix3d> estimate_pointwise_covariances(
            const std::vector<Eigen::Vector3d> &points,
            const KDTreeSearchParam &search_param = KDTreeSearchParamKNN());

    std::vector<Eigen::Vector3d> estimate_normals(
            const std::vector<Eigen::Matrix3d> &covariances);
}