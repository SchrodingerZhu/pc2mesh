#pragma once

#include <cstddef>
#include <algorithm>

namespace offload {
    struct Vector3D {
        double data[3];

        double &operator()(size_t x) {
            return data[x];
        }

        const double &operator()(size_t x) const {
            return data[x];
        }

        [[nodiscard]] Vector3D cross(const Vector3D &that) const {
            return {
                    this->data[1] * that.data[2] - this->data[2] * that.data[1],
                    -(this->data[0] * that.data[2] - this->data[2] * that.data[0]),
                    this->data[0] * that.data[1] - this->data[1] * that.data[0]
            };
        }

        [[nodiscard]] double dot(const Vector3D &that) const {
            return this->data[0] * that.data[0]
                   + this->data[1] * that.data[1]
                   + this->data[2] * that.data[2];
        }

        Vector3D operator/(double div) const {
            return {
                    this->data[0] / div,
                    this->data[1] / div,
                    this->data[2] / div
            };
        }

        Vector3D operator*(double mul) const {
            return {
                    this->data[0] * mul,
                    this->data[1] * mul,
                    this->data[2] * mul
            };
        }

        Vector3D operator-(const Vector3D &that) const {
            return {
                    this->data[0] - that.data[0],
                    this->data[1] - that.data[1],
                    this->data[2] - that.data[2]
            };
        }
    };

    struct Matrix3D {
        double data[3][3];

        const double &operator()(size_t x, size_t y) const {
            return data[x][y];
        }

        double &operator()(size_t x, size_t y) {
            return data[x][y];
        }

        [[nodiscard]] double max_element() const {
            auto max0 = std::max(std::max(data[0][0], data[0][1]), data[0][2]);
            auto max1 = std::max(std::max(data[1][0], data[1][1]), data[1][2]);
            auto max2 = std::max(std::max(data[2][0], data[2][1]), data[2][2]);
            return std::max(std::max(max0, max1), max2);
        }

        Matrix3D &operator/=(double div) {
            data[0][0] /= div;
            data[0][1] /= div;
            data[0][2] /= div;
            data[1][0] /= div;
            data[1][1] /= div;
            data[1][2] /= div;
            data[2][0] /= div;
            data[2][1] /= div;
            data[2][2] /= div;
            return *this;
        }

        Matrix3D &operator*=(double div) {
            data[0][0] *= div;
            data[0][1] *= div;
            data[0][2] *= div;
            data[1][0] *= div;
            data[1][1] *= div;
            data[1][2] *= div;
            data[2][0] *= div;
            data[2][1] *= div;
            data[2][2] *= div;
            return *this;
        }
    };

    void estimate_normals(size_t count, const Matrix3D * covariances, Vector3D* normals);
}