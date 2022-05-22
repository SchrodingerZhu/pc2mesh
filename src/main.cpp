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
    auto cloud = pc2mesh::geometry::PointCloud { std::move(data) };
    Eigen::Vector3d sum{};
    for (const auto &i: cloud.normals) {
        sum += i;
    }
    std::cout << sum << std::endl;

}