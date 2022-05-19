#include <iostream>
#include <chrono>
#include <pc2mesh/geometry/point_cloud.hpp>

int main() {
    std::vector<Eigen::Vector3d> data {};
    for (int i = 0; i < 10000; ++i) {
        data.emplace_back(rand(), rand(), rand());
        data.back().normalize();
    }
    auto normals = pc2mesh::geometry::estimate_normals(data);
    double sum {};
    for (const auto & i : normals) {
        sum += i.norm();
    }
    std::cout << sum << std::endl;

}