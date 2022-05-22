#include <iostream>
#include <chrono>
#include <pc2mesh/geometry/point_cloud.hpp>
#include <pc2mesh/geometry/intersection.hpp>
#include <pc2mesh/geometry/ball_pivoting.hpp>
#include <pc2mesh/geometry/triangle_mesh2dae.hpp>

#include <pc2mesh/geometry/poisson.hpp>
#include <pc2mesh/utilities/timer.hpp>

#include <pc2mesh/utilities/ply_parser.hpp>
int main(int argc, char** argv) {
    using pc2mesh::utilities::time;
    auto data = pc2mesh::utilities::load_ply(argv[1]);
    auto cloud = pc2mesh::geometry::PointCloud { data };

    int chosen = 0;
    if (argc > 2) {
        chosen = std::stoi(argv[2]);
    }
    if (chosen == 0) {
        auto trimesh = pc2mesh::geometry::create_triangle_mesh_ball_pivoting(cloud, {
                0.001, 0.01, 0.02, 0.04, 0.08, 0.16,
        });
        pc2mesh::geometry::tri2dae(cloud, trimesh);
    } else {
        auto trimesh = pc2mesh::geometry::create_triangle_mesh_possion(cloud, 8, 0, 1.1, true);
        pc2mesh::geometry::tri2dae(cloud, std::get<0>(trimesh), data.size());
    }
}
