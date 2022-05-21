//
// Created by schrodinger on 5/21/22.
//
#ifndef PLY_PARSER_H
#define PLY_PARSER_H

#include <rply.h>
#include <vector>
#include <eigen3/Eigen/Core>

namespace pc2mesh::utilities {
    namespace detail {
        struct InputData {
            std::vector<Eigen::Vector3d> buffer;
            Eigen::Vector3d current{};
        };

        static int callback(p_ply_argument argument) {
            InputData *data;
            long axis = 0;
            ply_get_argument_user_data(argument, reinterpret_cast<void **>(&data), &axis);
            data->current[axis] = ply_get_argument_value(argument);
            if(axis == 2) {
                data->buffer.push_back(data->current);
            }
            return 1;
        }
    }

    std::vector<Eigen::Vector3d> load_ply(const char *ply_file) {
        detail::InputData data;
        p_ply ply = ply_open(ply_file, nullptr, 0, nullptr);
        ply_read_header(ply);
        auto size = ply_set_read_cb(ply, "vertex", "x", detail::callback, &data, 0);
        ply_set_read_cb(ply, "vertex", "y", detail::callback, &data, 1);
        ply_set_read_cb(ply, "vertex", "z", detail::callback, &data, 2);
        data.buffer.reserve(size);
        ply_read(ply); // TODO: Error
        ply_close(ply);
        return data.buffer;
    }
}

#endif // PLY_PARSER_H
