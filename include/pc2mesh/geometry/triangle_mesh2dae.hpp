//
// Created by polo on 5/22/22.
//

#ifndef PC2MESH_TRIANGLE_MESH2DAE_HPP
#define PC2MESH_TRIANGLE_MESH2DAE_HPP

#include <iostream>
#include <pc2mesh/geometry/triangle_mesh.hpp>

#endif //PC2MESH_TRIANGLE_MESH2DAE_HPP


namespace pc2mesh::geometry {

    void tri2dae (const char* output, const PointCloud &pcd, const TriangleMesh &trimesh, size_t index_shift = 0);



}
