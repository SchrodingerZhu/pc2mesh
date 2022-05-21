#pragma once
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
    }
}