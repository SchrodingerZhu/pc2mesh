#ifndef UTIL_H
#define UTIL_H

#include "rply.h"
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;

Eigen::Vector3d getTriNormal(Eigen::Vector3d triangle, const vector<Eigen::Vector3d> &vertices);


#endif // UTIL_H
