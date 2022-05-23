#include <iostream>
#include <pc2mesh/geometry/point_cloud.hpp>
#include <pc2mesh/cli.h>
#include <pc2mesh/geometry/ball_pivoting.hpp>
#include <pc2mesh/geometry/triangle_mesh2dae.hpp>

#include <pc2mesh/geometry/poisson.hpp>
#include <pc2mesh/utilities/timer.hpp>

#include <pc2mesh/utilities/ply_parser.hpp>

int main() {
    auto cli_data = pc2mesh_create_cli();
    if (cli_data.is_poisson) {
        auto data = pc2mesh::utilities::load_ply(cli_data.poisson.input_file);
        auto cloud = pc2mesh::geometry::PointCloud{data};
        auto trimesh =
                pc2mesh::geometry::create_triangle_mesh_possion(
                        cloud,
                        cli_data.poisson.depth,
                        static_cast<float>(cli_data.poisson.width),
                        static_cast<float>(cli_data.poisson.scale),
                        cli_data.poisson.linear_fit,
                        static_cast<int>(cli_data.poisson.num_threads),
                        cli_data.poisson.filter);
        pc2mesh::geometry::tri2dae(cli_data.poisson.output_file, cloud, trimesh, data.size());
    } else {
        auto data = pc2mesh::utilities::load_ply(cli_data.poisson.input_file);
        auto cloud = pc2mesh::geometry::PointCloud{data};
        auto trimesh = pc2mesh::geometry::create_triangle_mesh_ball_pivoting(
                cloud,
                std::vector(cli_data.ball_pivoting.radii,
                            cli_data.ball_pivoting.radii + cli_data.ball_pivoting.radii_size)
        );
        pc2mesh::geometry::tri2dae(cli_data.poisson.output_file, cloud, trimesh, data.size());
    }
}
