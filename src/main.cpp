#include <iostream>
#include <chrono>
#include <pc2mesh/geometry/point_cloud.hpp>

int main() {
    std::vector<Eigen::Vector3d> data {
            {0, 0, 0},
            {0, -2, 0},
            {2, 0, 0},
    };
    for (int i = 0; i < 1'000'000; ++i) {
        data.emplace_back(rand(), rand(), rand());
    }
    auto normals = pc2mesh::geometry::estimate_normals(data);
}