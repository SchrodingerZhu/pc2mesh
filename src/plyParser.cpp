#include "plyParser.h"
#include <iostream>



static double *verticeBuffer;
static unsigned int *faceBuffer;
static int vertIdx;
static int faceIdx;

static int vertex_cb(p_ply_argument argument) {
    long eol;
    ply_get_argument_user_data(argument, NULL, &eol);
    double value = ply_get_argument_value(argument);
    verticeBuffer[vertIdx] = value;
    ++vertIdx;
    return 1;
}

static int face_cb(p_ply_argument argument) {
    long length, value_index;
    ply_get_argument_property(argument, NULL, &length, &value_index);
    if (value_index == 0 || value_index == 1 || value_index == 2) {
        unsigned int idx = (unsigned int) ply_get_argument_value(argument);
        faceBuffer[faceIdx] = idx;
        ++faceIdx;
    }
    return 1;
}

plyParser::plyParser(){

}


int plyParser::readPly(char *ply_file, vector<Eigen::Vector3d> &vertices, vector<Eigen::Vector3d> &triangles){
    free(verticeBuffer);
    free(faceBuffer);
    long nVertices, nTriangles;

    p_ply ply = ply_open(ply_file, NULL, 0, NULL);

    if((!ply) && (!ply_read_header(ply))){
        std::cout << "Failed to read file!" << std::endl;
        return 1;
    }

    nVertices = ply_set_read_cb(ply, "vertex", "x", vertex_cb, NULL, 0);
    ply_set_read_cb(ply, "vertex", "y", vertex_cb, NULL, 0);
    ply_set_read_cb(ply, "vertex", "z", vertex_cb, NULL, 1);
    
    nTriangles = ply_set_read_cb(ply, "face", "vertex_indices", face_cb, NULL, 0);

    verticeBuffer = (double*) malloc(sizeof(double) * nVertices * 3); // each vertices has x,y,z 3 values
    faceBuffer = (unsigned int*) malloc(sizeof(unsigned int) * nTriangles * 3); // each trtiangle has indices of 3 vertices

    if(!ply_read(ply)) return 1;
    ply_close(ply);

    // load data to the vector
    for(int i = 0; i < nVertices; i++){
        Eigen::Vector3d v_temp(verticeBuffer[i*3], verticeBuffer[i*3+1], verticeBuffer[i*3+2]);
        vertices.push_back(v_temp);
    }

    for(int j = 0; j < nTriangles; j++){
        Eigen::Vector3d f_temp(faceBuffer[j*3], faceBuffer[j*3+1], faceBuffer[j*3+2]);
        vertices.push_back(f_temp);
    }

    free(verticeBuffer);
    free(faceBuffer);
    verticeBuffer = NULL;
    faceBuffer = NULL;
    return 0;
}