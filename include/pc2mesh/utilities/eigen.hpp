#pragma once
#include <vector>
#include <Eigen/Core>
namespace pc2mesh::utilities {
    template<typename PtContainer, typename IdxContainer>
    __attribute__((always_inline)) inline Eigen::Matrix3d ComputeCovariance(const PtContainer &points,
                                                                     const IdxContainer &indices) {
        if (indices.empty()) {
            return Eigen::Matrix3d::Identity();
        }
        Eigen::Matrix3d covariance;
        Eigen::Matrix<double, 9, 1> cumulants;
        cumulants.setZero();
        for (const auto &idx: indices) {
            const Eigen::Vector3d &point = points[idx];
            cumulants(0) += point(0);
            cumulants(1) += point(1);
            cumulants(2) += point(2);
            cumulants(3) += point(0) * point(0);
            cumulants(4) += point(0) * point(1);
            cumulants(5) += point(0) * point(2);
            cumulants(6) += point(1) * point(1);
            cumulants(7) += point(1) * point(2);
            cumulants(8) += point(2) * point(2);
        }
        cumulants /= (double) indices.size();
        covariance(0, 0) = cumulants(3) - cumulants(0) * cumulants(0);
        covariance(1, 1) = cumulants(6) - cumulants(1) * cumulants(1);
        covariance(2, 2) = cumulants(8) - cumulants(2) * cumulants(2);
        covariance(0, 1) = cumulants(4) - cumulants(0) * cumulants(1);
        covariance(1, 0) = covariance(0, 1);
        covariance(0, 2) = cumulants(5) - cumulants(0) * cumulants(2);
        covariance(2, 0) = covariance(0, 2);
        covariance(1, 2) = cumulants(7) - cumulants(1) * cumulants(2);
        covariance(2, 1) = covariance(1, 2);
        return covariance;
    }

    template <typename H>
    __attribute__((always_inline)) inline H AbslHashValue(H h, const Eigen::Vector3d& m) {
        return H::combine_contiguous(h, m.data(), m.size());
    }
}
