// Modified from Open3D
// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018-2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4267)
#endif

#include <pc2mesh/geometry/kdtree_flann.hpp>

#include <nanoflann.hpp>

namespace pc2mesh {
    namespace geometry {

        KDTreeFlann::KDTreeFlann() {}

        KDTreeFlann::KDTreeFlann(const Eigen::MatrixXd &data) { SetMatrixData(data); }


        KDTreeFlann::~KDTreeFlann() {}

        bool KDTreeFlann::SetMatrixData(const Eigen::MatrixXd &data) {
            return SetRawData(Eigen::Map<const Eigen::MatrixXd>(
                    data.data(), data.rows(), data.cols()));
        }

        bool KDTreeFlann::SetRawData(const Eigen::Map<const Eigen::MatrixXd> &data) {
            dimension_ = data.rows();
            dataset_size_ = data.cols();
            if (dimension_ == 0 || dataset_size_ == 0) {
                return false;
            }
            data_.resize(dataset_size_ * dimension_);
            memcpy(data_.data(), data.data(),
                   dataset_size_ * dimension_ * sizeof(double));
            data_interface_.reset(new Eigen::Map<const Eigen::MatrixXd>(data));
            nanoflann_index_.reset(
                    new KDTree_t(dimension_, std::cref(*data_interface_), 15));
            nanoflann_index_->index->buildIndex();
            return true;
        }

    }  // namespace geometry
}  // namespace pc2mesh

#ifdef _MSC_VER
#pragma warning(pop)
#endif

