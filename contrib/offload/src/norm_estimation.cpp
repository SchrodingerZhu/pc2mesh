#include <offload/norm_estimation.hpp>
#include <cmath>
#include <iostream>

namespace offload {
    namespace detail {
        __attribute__((always_inline)) static inline Vector3D compute_eigenvector0(const Matrix3D &A, double eval0) {
            Vector3D row0{A(0, 0) - eval0, A(0, 1), A(0, 2)};
            Vector3D row1{A(0, 1), A(1, 1) - eval0, A(1, 2)};
            Vector3D row2{A(0, 2), A(1, 2), A(2, 2) - eval0};
            Vector3D r0xr1 = row0.cross(row1);
            Vector3D r0xr2 = row0.cross(row2);
            Vector3D r1xr2 = row1.cross(row2);
            double d0 = r0xr1.dot(r0xr1);
            double d1 = r0xr2.dot(r0xr2);
            double d2 = r1xr2.dot(r1xr2);

            double dmax = d0;
            int imax = 0;
            if (d1 > dmax) {
                dmax = d1;
                imax = 1;
            }
            if (d2 > dmax) {
                imax = 2;
            }

            if (imax == 0) {
                return r0xr1 / std::sqrt(d0);
            } else if (imax == 1) {
                return r0xr2 / std::sqrt(d1);
            } else {
                return r1xr2 / std::sqrt(d2);
            }
        }

        __attribute__((always_inline)) static inline Vector3D compute_eigenvector1(const Matrix3D &A,
                                                                                   const Vector3D &evec0,
                                                                                   double eval1) {
            Vector3D U{}, V{};
            if (std::abs(evec0(0)) > std::abs(evec0(1))) {
                double inv_length =
                        1 / std::sqrt(evec0(0) * evec0(0) + evec0(2) * evec0(2));
                U = Vector3D{-evec0(2) * inv_length, 0, evec0(0) * inv_length};
            } else {
                double inv_length =
                        1 / std::sqrt(evec0(1) * evec0(1) + evec0(2) * evec0(2));
                U = Vector3D{0, evec0(2) * inv_length, -evec0(1) * inv_length};
            }
            V = evec0.cross(U);

            Vector3D AU{A(0, 0) * U(0) + A(0, 1) * U(1) + A(0, 2) * U(2),
                        A(0, 1) * U(0) + A(1, 1) * U(1) + A(1, 2) * U(2),
                        A(0, 2) * U(0) + A(1, 2) * U(1) + A(2, 2) * U(2)};

            Vector3D AV{A(0, 0) * V(0) + A(0, 1) * V(1) + A(0, 2) * V(2),
                        A(0, 1) * V(0) + A(1, 1) * V(1) + A(1, 2) * V(2),
                        A(0, 2) * V(0) + A(1, 2) * V(1) + A(2, 2) * V(2)};

            double m00 = U(0) * AU(0) + U(1) * AU(1) + U(2) * AU(2) - eval1;
            double m01 = U(0) * AV(0) + U(1) * AV(1) + U(2) * AV(2);
            double m11 = V(0) * AV(0) + V(1) * AV(1) + V(2) * AV(2) - eval1;

            double absM00 = std::abs(m00);
            double absM01 = std::abs(m01);
            double absM11 = std::abs(m11);
            double max_abs_comp;
            if (absM00 >= absM11) {
                max_abs_comp = std::max(absM00, absM01);
                if (max_abs_comp > 0) {
                    if (absM00 >= absM01) {
                        m01 /= m00;
                        m00 = 1 / std::sqrt(1 + m01 * m01);
                        m01 *= m00;
                    } else {
                        m00 /= m01;
                        m01 = 1 / std::sqrt(1 + m00 * m00);
                        m00 *= m01;
                    }
                    return U * m01 - V * m00;
                } else {
                    return U;
                }
            } else {
                max_abs_comp = std::max(absM11, absM01);
                if (max_abs_comp > 0) {
                    if (absM11 >= absM01) {
                        m01 /= m11;
                        m11 = 1 / std::sqrt(1 + m01 * m01);
                        m01 *= m11;
                    } else {
                        m11 /= m01;
                        m01 = 1 / std::sqrt(1 + m11 * m11);
                        m11 *= m01;
                    }
                    return U * m11 - V * m01;
                } else {
                    return U;
                }
            }
        }

        __attribute__((always_inline)) static inline Vector3D fast_eigen_3x3(const Matrix3D &covariance) {
            // Previous version based on:
            // https://en.wikipedia.org/wiki/Eigenvalue_algorithm#3.C3.973_matrices
            // Current version based on
            // https://www.geometrictools.com/Documentation/RobustEigenSymmetric3x3.pdf
            // which handles edge cases like points on a plane

            Matrix3D A = covariance;
            double max_coeff = A.max_element();
            if (max_coeff <= 0) {
                return Vector3D{};
            }

            double norm = A(0, 1) * A(0, 1) + A(0, 2) * A(0, 2) + A(1, 2) * A(1, 2);
            if (norm > 0) {
                Vector3D eval{};
                Vector3D evec0{};
                Vector3D evec1{};
                Vector3D evec2{};

                double q = (A(0, 0) + A(1, 1) + A(2, 2)) / 3;

                double b00 = A(0, 0) - q;
                double b11 = A(1, 1) - q;
                double b22 = A(2, 2) - q;

                double p =
                        std::sqrt((b00 * b00 + b11 * b11 + b22 * b22 + norm * 2) / 6);

                double c00 = b11 * b22 - A(1, 2) * A(1, 2);
                double c01 = A(0, 1) * b22 - A(1, 2) * A(0, 2);
                double c02 = A(0, 1) * A(1, 2) - b11 * A(0, 2);
                double det = (b00 * c00 - A(0, 1) * c01 + A(0, 2) * c02) / (p * p * p);

                double half_det = det * 0.5;
                half_det = std::min(std::max(half_det, -1.0), 1.0);

                double angle = std::acos(half_det) / (double) 3;
                double const two_thirds_pi = 2.09439510239319549;
                double beta2 = std::cos(angle) * 2;
                double beta0 = std::cos(angle + two_thirds_pi) * 2;
                double beta1 = -(beta0 + beta2);

                eval(0) = q + p * beta0;
                eval(1) = q + p * beta1;
                eval(2) = q + p * beta2;

                if (half_det >= 0) {
                    evec2 = compute_eigenvector0(A, eval(2));
                    if (eval(2) < eval(0) && eval(2) < eval(1)) {
                        return evec2;
                    }
                    evec1 = compute_eigenvector1(A, evec2, eval(1));
                    if (eval(1) < eval(0) && eval(1) < eval(2)) {
                        return evec1;
                    }
                    evec0 = evec1.cross(evec2);
                    return evec0;
                } else {
                    evec0 = compute_eigenvector0(A, eval(0));
                    if (eval(0) < eval(1) && eval(0) < eval(2)) {
                        return evec0;
                    }
                    evec1 = compute_eigenvector1(A, evec0, eval(1));
                    if (eval(1) < eval(0) && eval(1) < eval(2)) {
                        return evec1;
                    }
                    evec2 = evec0.cross(evec1);
                    return evec2;
                }
            } else {
                if (A(0, 0) < A(1, 1) && A(0, 0) < A(2, 2)) {
                    return Vector3D{1, 0, 0};
                } else if (A(1, 1) < A(0, 0) && A(1, 1) < A(2, 2)) {
                    return Vector3D{0, 1, 0};
                } else {
                    return Vector3D{0, 0, 1};
                }
            }
        }
    }

    void estimate_normals(size_t count, const Matrix3D *covariances, Vector3D *normals) {
#pragma omp target map(to: covariances[0:count]) map(from: normals[0:count])
#pragma omp teams distribute parallel for default(none) shared(count, covariances, normals)
        for (size_t i = 0; i < count; i++) {
            normals[i] = detail::fast_eigen_3x3(covariances[i]);
        }
    }

    Vector3D estimate_normal_from_cov(
            const Matrix3D &cov
    ) {
        return detail::fast_eigen_3x3(cov);
    }

}