#ifndef PLY_PARSER_H
#define PLY_PARSER_H

#include "rply.h"
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;

class plyParser {
public:
    plyParser();
    int readPly(char *ply_file, vector<Eigen::Vector3d> &vertices, vector<Eigen::Vector3d> &triangles);
    // int parsePly(char *ifile, vector<Eigen::Vector3d> &vertices, vector<Eigen::Vector3d> &normals);
    // int parsePlyFaces(char *ifile, vector<Eigen::Vector3d> &vertices, vector<Eigen::Vector3d> &triangles);
    // int parsePlyLines(char *ifile, vector<Eigen::Vector3d> &vertices, vector<pair<int, int> > &lines);
    // int writeTxt(char *ofile, vector<Vector3D> &vertices, vector<Vector3D> &normals);
    // int parseTxt(const char *ifile, vector<Vector3D> &vertices, vector<Vector3D> &normals);
private:

};

#endif // PLY_PARSER_H
