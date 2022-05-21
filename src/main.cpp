#include <iostream>
#include <chrono>
#include <pc2mesh/geometry/point_cloud.hpp>
#include <pc2mesh/geometry/intersection.hpp>
#include <pc2mesh/geometry/ball_pivoting.hpp>
#include <pc2mesh/utilities/timer.hpp>
#include <pc2mesh/utilities/ply_parser.hpp>

int main(int argc, char** argv) {
    using pc2mesh::utilities::time;
    auto data = pc2mesh::utilities::load_ply(argv[1]);
    for (const auto &i: data) {
        std::cout << i << std::endl << std::endl;
    }
    auto covs = time("covariance estimation", [&] { return pc2mesh::geometry::estimate_pointwise_covariances(data); });
    auto normals = time("normal estimation", [&] { return pc2mesh::geometry::estimate_normals(covs); });
    Eigen::Vector3d sum{};
    for (const auto &i: normals) {
        sum += i;
    }
    std::cout << sum << std::endl;

}