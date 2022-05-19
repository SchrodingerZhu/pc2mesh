#pragma once

#include <pc2mesh/geometry/kdtree_flann.hpp>

namespace pc2mesh::geometry {
    std::vector<Eigen::Vector3d> estimate_normals(
            const std::vector<Eigen::Vector3d> &points,
            const KDTreeSearchParam &search_param = KDTreeSearchParamKNN());
}