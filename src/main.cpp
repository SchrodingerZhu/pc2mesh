#include <iostream>
#include <chrono>
#include <pc2mesh/geometry/point_cloud.hpp>
#include <pc2mesh/utilities/timer.hpp>

int main() {
    using pc2mesh::utilities::time;
    std::vector<Eigen::Vector3d> data{};
    for (int i = 0; i < 1000000; ++i) {
        data.emplace_back(rand(), rand(), rand());
    }
    auto covs = time("covariance estimation", [&] { return pc2mesh::geometry::estimate_pointwise_covariances(data); });
    auto normals = time("normal estimation", [&] { return pc2mesh::geometry::estimate_normals(covs); });
    Eigen::Vector3d sum{};
    for (const auto &i: normals) {
        sum += i;
    }
    std::cout << sum << std::endl;

}