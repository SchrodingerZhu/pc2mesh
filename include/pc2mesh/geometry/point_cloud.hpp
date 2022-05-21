#pragma once

#include <pc2mesh/geometry/kdtree_param.hpp>
#include <Eigen/Eigen>
#include <memory>
namespace pc2mesh::geometry {
    std::vector<Eigen::Matrix3d> estimate_pointwise_covariances(
            const class KDTreeFlann& kd_tree,
            const std::vector<Eigen::Vector3d> &points,
            const KDTreeSearchParam &search_param = KDTreeSearchParamKNN());

    std::vector<Eigen::Vector3d> estimate_normals(
            const std::vector<Eigen::Matrix3d> &covariances);

    struct PointCloud {
        explicit PointCloud(std::vector<Eigen::Vector3d> points);

        std::shared_ptr<class KDTreeFlann> kdtree;
        std::vector<Eigen::Vector3d> points;
        std::vector<Eigen::Matrix3d> covariances;
        std::vector<Eigen::Vector3d> normals;
    };
}