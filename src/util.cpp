#include "util.h"

using namespace Eigen;


Eigen::Vector3d getTriNormal(Eigen::Vector3d triangle, const vector<Eigen::Vector3d> &vertices){
    int i1 = triangle[0];
    int i2 = triangle[1];
    int i3 = triangle[2];

    Eigen::Vector3d v1 = vertices[i1];
    Eigen::Vector3d v2 = vertices[i2];
    Eigen::Vector3d v3 = vertices[i3];

    Eigen::Vector3d v12 = v2 - v1;
    Eigen::Vector3d v13 = v3 - v1;

    Eigen::Vector3d n = v12.cross(v13);

    return n;
}