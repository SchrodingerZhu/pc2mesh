#ifndef PLY_PARSER_H
#define PLY_PARSER_H
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace pc2mesh::utilities {
    std::vector<Eigen::Vector3d> load_ply(const char *ply_file);
}

#endif // PLY_PARSER_H
