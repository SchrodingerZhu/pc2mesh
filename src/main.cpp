#include <iostream>
#include <chrono>
#include <pc2mesh/geometry/point_cloud.hpp>
#include <pc2mesh/geometry/intersection.hpp>
#include <pc2mesh/geometry/ball_pivoting.hpp>
#include <pc2mesh/geometry/poisson.hpp>
#include <pc2mesh/utilities/timer.hpp>
#include <pc2mesh/utilities/ply_parser.hpp>

int main(int argc, char** argv) {
    using pc2mesh::utilities::time;
    auto data = pc2mesh::utilities::load_ply(argv[1]);
    auto cloud = pc2mesh::geometry::PointCloud { std::move(data) };
    auto trimesh = pc2mesh::geometry::create_triangle_mesh_ball_pivoting(cloud, {
       0.001, 0.01, 0.02, 0.04, 0.08, 0.16, 0.5, 1,
    });
//    auto trimeshp = pc2mesh::geometry::create_triangle_mesh_possion(cloud);

    Eigen::Vector3d sum{};
    for (const auto &i: trimesh.triangle_normals) {
        std::cout << i << std::endl << std::endl;
    }

//    sum = {0, 0, 0};
//    for (const auto &i: std::get<0>(trimeshp).triangle_normals) {
//        std::cout << i << std::endl << std::endl;
//    }

}