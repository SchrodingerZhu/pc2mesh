#include <Eigen/Core>
#include <tomasakeninemoeller/tribox3.h>
#include <tomasakeninemoeller/opttritri.h>

namespace pc2mesh::geometry {
    __attribute__((always_inline)) static inline bool boxes_intersection(const Eigen::Vector3d &min0,
                                                                         const Eigen::Vector3d &max0,
                                                                         const Eigen::Vector3d &min1,
                                                                         const Eigen::Vector3d &max1) {
        if (max0(0) < min1(0) || min0(0) > max1(0)) {
            return false;
        }
        if (max0(1) < min1(1) || min0(1) > max1(1)) {
            return false;
        }
        if (max0(2) < min1(2) || min0(2) > max1(2)) {
            return false;
        }
        return true;
    }

    __attribute__((always_inline)) static inline bool triangle_3d_intersection(const Eigen::Vector3d &p0,
                                                                               const Eigen::Vector3d &p1,
                                                                               const Eigen::Vector3d &p2,
                                                                               const Eigen::Vector3d &q0,
                                                                               const Eigen::Vector3d &q1,
                                                                               const Eigen::Vector3d &q2) {
        const Eigen::Vector3d mu = (p0 + p1 + p2 + q0 + q1 + q2) / 6;
        const Eigen::Vector3d sigma =
                (((p0 - mu).array().square() + (p1 - mu).array().square() +
                  (p2 - mu).array().square() + (q0 - mu).array().square() +
                  (q1 - mu).array().square() + (q2 - mu).array().square()) /
                 5)
                        .sqrt() +
                1e-12;
        Eigen::Vector3d p0m = (p0 - mu).array() / sigma.array();
        Eigen::Vector3d p1m = (p1 - mu).array() / sigma.array();
        Eigen::Vector3d p2m = (p2 - mu).array() / sigma.array();
        Eigen::Vector3d q0m = (q0 - mu).array() / sigma.array();
        Eigen::Vector3d q1m = (q1 - mu).array() / sigma.array();
        Eigen::Vector3d q2m = (q2 - mu).array() / sigma.array();
        return tomasakeninemoeller::NoDivTriTriIsect(p0m.data(), p1m.data(), p2m.data(), q0m.data(),
                                                     q1m.data(), q2m.data()) != 0;
    }
}