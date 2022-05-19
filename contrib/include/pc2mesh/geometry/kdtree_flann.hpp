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

#pragma once

#include <Eigen/Core>
#include <nanoflann.hpp>
#include <memory>

namespace pc2mesh {
    namespace geometry {


        class KDTreeSearchParam {
        public:
            enum class SearchType {
                Knn = 0,
                Radius = 1,
                Hybrid = 2,
            };

        public:
            virtual ~KDTreeSearchParam() = default;

        protected:
            explicit KDTreeSearchParam(SearchType type) : search_type_(type) {}

        public:
            SearchType GetSearchType() const { return search_type_; }

        private:
            SearchType search_type_;
        };

        class KDTreeSearchParamKNN : public KDTreeSearchParam {
        public:
            explicit KDTreeSearchParamKNN(int knn = 30)
                    : KDTreeSearchParam(SearchType::Knn), knn_(knn) {}

        public:
            int knn_;
        };

        class KDTreeSearchParamRadius : public KDTreeSearchParam {
        public:
            explicit KDTreeSearchParamRadius(double radius)
                    : KDTreeSearchParam(SearchType::Radius), radius_(radius) {}

        public:
            double radius_;
        };


        class KDTreeSearchParamHybrid : public KDTreeSearchParam {
        public:
            KDTreeSearchParamHybrid(double radius, int max_nn)
                    : KDTreeSearchParam(SearchType::Hybrid),
                      radius_(radius),
                      max_nn_(max_nn) {}

        public:
            double radius_;
            int max_nn_;
        };


        class KDTreeFlann {
        public:

            KDTreeFlann();

            explicit KDTreeFlann(const Eigen::MatrixXd &data);

            ~KDTreeFlann();

            KDTreeFlann(const KDTreeFlann &) = delete;

            KDTreeFlann &operator=(const KDTreeFlann &) = delete;

            bool SetRawData(const Eigen::Map<const Eigen::MatrixXd> &data);

        public:
            bool SetMatrixData(const Eigen::MatrixXd &data);

            template<typename T>
            int Search(const T &query,
                       const KDTreeSearchParam &param,
                       std::vector<int> &indices,
                       std::vector<double> &distance2) const;

            template<typename T>
            int SearchKNN(const T &query,
                          int knn,
                          std::vector<int> &indices,
                          std::vector<double> &distance2) const;

            template<typename T>
            int SearchRadius(const T &query,
                             double radius,
                             std::vector<int> &indices,
                             std::vector<double> &distance2) const;

            template<typename T>
            int SearchHybrid(const T &query,
                             double radius,
                             int max_nn,
                             std::vector<int> &indices,
                             std::vector<double> &distance2) const;

        protected:
            using KDTree_t = nanoflann::KDTreeEigenMatrixAdaptor<
                    Eigen::Map<const Eigen::MatrixXd>,
                    -1,
                    nanoflann::metric_L2,
                    false>;

            std::vector<double> data_;
            std::unique_ptr<Eigen::Map<const Eigen::MatrixXd>> data_interface_;
            std::unique_ptr<KDTree_t> nanoflann_index_;
            size_t dimension_ = 0;
            size_t dataset_size_ = 0;
        };

        template<typename T>
        int KDTreeFlann::Search(const T &query,
                                const KDTreeSearchParam &param,
                                std::vector<int> &indices,
                                std::vector<double> &distance2) const {
            switch (param.GetSearchType()) {
                case KDTreeSearchParam::SearchType::Knn:
                    return SearchKNN(query, ((const KDTreeSearchParamKNN &) param).knn_,
                                     indices, distance2);
                case KDTreeSearchParam::SearchType::Radius:
                    return SearchRadius(
                            query, ((const KDTreeSearchParamRadius &) param).radius_,
                            indices, distance2);
                case KDTreeSearchParam::SearchType::Hybrid:
                    return SearchHybrid(
                            query, ((const KDTreeSearchParamHybrid &) param).radius_,
                            ((const KDTreeSearchParamHybrid &) param).max_nn_, indices,
                            distance2);
                default:
                    return -1;
            }
            return -1;
        }

        template<typename T>
        int KDTreeFlann::SearchKNN(const T &query,
                                   int knn,
                                   std::vector<int> &indices,
                                   std::vector<double> &distance2) const {
            // This is optimized code for heavily repeated search.
            // Other flann::Index::knnSearch() implementations lose performance due to
            // memory allocation/deallocation.
            if (data_.empty() || dataset_size_ <= 0 ||
                size_t(query.rows()) != dimension_ || knn < 0) {
                return -1;
            }
            indices.resize(knn);
            distance2.resize(knn);
            std::vector<Eigen::Index> indices_eigen(knn);
            int k = nanoflann_index_->index->knnSearch(
                    query.data(), knn, indices_eigen.data(), distance2.data());
            indices.resize(k);
            distance2.resize(k);
            std::copy_n(indices_eigen.begin(), k, indices.begin());
            return k;
        }

        template<typename T>
        int KDTreeFlann::SearchRadius(const T &query,
                                      double radius,
                                      std::vector<int> &indices,
                                      std::vector<double> &distance2) const {
            // This is optimized code for heavily repeated search.
            // Since max_nn is not given, we let flann to do its own memory management.
            // Other flann::Index::radiusSearch() implementations lose performance due
            // to memory management and CPU caching.
            if (data_.empty() || dataset_size_ <= 0 ||
                size_t(query.rows()) != dimension_) {
                return -1;
            }
            std::vector<std::pair<Eigen::Index, double>> indices_dists;
            int k = nanoflann_index_->index->radiusSearch(
                    query.data(), radius * radius, indices_dists,
                    nanoflann::SearchParams(-1, 0.0));
            indices.resize(k);
            distance2.resize(k);
            for (int i = 0; i < k; ++i) {
                indices[i] = indices_dists[i].first;
                distance2[i] = indices_dists[i].second;
            }
            return k;
        }

        template<typename T>
        int KDTreeFlann::SearchHybrid(const T &query,
                                      double radius,
                                      int max_nn,
                                      std::vector<int> &indices,
                                      std::vector<double> &distance2) const {
            // This is optimized code for heavily repeated search.
            // It is also the recommended setting for search.
            // Other flann::Index::radiusSearch() implementations lose performance due
            // to memory allocation/deallocation.
            if (data_.empty() || dataset_size_ <= 0 ||
                size_t(query.rows()) != dimension_ || max_nn < 0) {
                return -1;
            }
            distance2.resize(max_nn);
            std::vector<Eigen::Index> indices_eigen(max_nn);
            int k = nanoflann_index_->index->knnSearch(
                    query.data(), max_nn, indices_eigen.data(), distance2.data());
            k = std::distance(distance2.begin(),
                              std::lower_bound(distance2.begin(), distance2.begin() + k,
                                               radius * radius));
            indices.resize(k);
            distance2.resize(k);
            std::copy_n(indices_eigen.begin(), k, indices.begin());
            return k;
        }
    }  // namespace geometry
}  // namespace pc2mesh