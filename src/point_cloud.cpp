#include <pc2mesh/utilities/eigen.h>
#include <pc2mesh/geometry/kdtree_flann.hpp>
#include <pc2mesh/geometry/point_cloud.hpp>
#include <offload/norm_estimation.hpp>
#include <nanoflann.hpp>
#include <iostream>
namespace pc2mesh::geometry {
    std::vector<Eigen::Matrix3d> EstimatePerPointCovariances(
            const std::vector<Eigen::Vector3d> &points,
            const KDTreeSearchParam &search_param /* = KDTreeSearchParamKNN()*/) {
        std::vector<Eigen::Matrix3d> covariances;
        covariances.resize(points.size());

        KDTreeFlann kdtree;
        kdtree.SetRawData(Eigen::Map<const Eigen::MatrixXd>(
                (const double *) points.data(),
                3, static_cast<Eigen::Index>(points.size())));
#pragma omp parallel for schedule(static) default(none) shared(kdtree, points, search_param, covariances)
        for (int i = 0; i < (int) points.size(); i++) {
            std::vector<int> indices;
            std::vector<double> distance2;
            if (kdtree.Search(points[i], search_param, indices, distance2) >= 3) {
                auto covariance = utilities::ComputeCovariance(points, indices);
                covariances[i] = covariance;
            } else {
                covariances[i] = Eigen::Matrix3d::Identity();
            }
        }
        return covariances;
    }

    std::vector<Eigen::Vector3d> estimate_normals(
            const std::vector<Eigen::Vector3d> &points,
            const KDTreeSearchParam &search_param /* = KDTreeSearchParamKNN()*/) {

        auto cov_start = std::chrono::high_resolution_clock::now();
        auto covariances = EstimatePerPointCovariances(points, search_param);
        auto cov_end = std::chrono::high_resolution_clock::now();
        std::cout << "cov calculated in " << std::chrono::duration_cast<std::chrono::milliseconds>(cov_end - cov_start).count() << "ms" << std::endl;

        std::vector<offload::Matrix3D> transformed(covariances.size());
        std::vector<offload::Vector3D> normals(covariances.size());
        std::transform(covariances.begin(), covariances.end(), transformed.begin(), [](const Eigen::Matrix3d &data) {
            return offload::Matrix3D{
                    data(0, 0), data(0, 1), data(0, 2),
                    data(1, 0), data(1, 1), data(1, 2),
                    data(2, 0), data(2, 1), data(2, 2)
            };
        });
        auto norm_start = std::chrono::high_resolution_clock::now();
        offload::estimate_normals(transformed.size(), transformed.data(), normals.data());
        auto norm_end = std::chrono::high_resolution_clock::now();
        std::cout << "norm calculated in " << std::chrono::duration_cast<std::chrono::milliseconds>(norm_end - norm_start).count() << "ms" << std::endl;

        std::vector<Eigen::Vector3d> result(covariances.size());
        std::transform(normals.begin(), normals.end(), result.begin(), [](const offload::Vector3D &data) {
            return Eigen::Vector3d {
                data(0), data(1), data(2)
            };
        });
        return result;
    }
}