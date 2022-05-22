#include <deque>
#include <list>

#include <pc2mesh/geometry/intersection.hpp>
#include <pc2mesh/geometry/kdtree_flann.hpp>
#include <pc2mesh/geometry/triangle_mesh.hpp>
#include <pc2mesh/geometry/point_cloud.hpp>
#include <pc2mesh/geometry/ball_pivoting.hpp>
#include <absl/container/flat_hash_set.h>


namespace pc2mesh::geometry {

    class BallPivotingVertex;

    class BallPivotingEdge;

    class BallPivotingTriangle;

    using BallPivotingVertexPtr = BallPivotingVertex *;
    using BallPivotingEdgePtr = std::shared_ptr<BallPivotingEdge>;
    using BallPivotingTrianglePtr = std::shared_ptr<BallPivotingTriangle>;

    class BallPivotingVertex {
    public:
        enum Type {
            Orphan = 0, Front = 1, Inner = 2
        };

        BallPivotingVertex(int idx,
                           const Eigen::Vector3d &point,
                           const Eigen::Vector3d &normal)
                : idx_(idx), point_(point), normal_(normal), type_(Orphan) {}

        void UpdateType();

    public:
        int idx_;
        const Eigen::Vector3d &point_;
        const Eigen::Vector3d &normal_;
        absl::flat_hash_set<BallPivotingEdgePtr> edges_;
        Type type_;
    };

    class BallPivotingEdge {
    public:
        enum Type {
            Border = 0, Front = 1, Inner = 2
        };

        BallPivotingEdge(BallPivotingVertexPtr source, BallPivotingVertexPtr target)
                : source_(source), target_(target), type_(Type::Front) {}

        void AddAdjacentTriangle(BallPivotingTrianglePtr triangle);

        BallPivotingVertexPtr GetOppositeVertex();

    public:
        BallPivotingVertexPtr source_;
        BallPivotingVertexPtr target_;
        BallPivotingTrianglePtr triangle0_;
        BallPivotingTrianglePtr triangle1_;
        Type type_;
    };

    class BallPivotingTriangle {
    public:
        BallPivotingTriangle(BallPivotingVertexPtr vert0,
                             BallPivotingVertexPtr vert1,
                             BallPivotingVertexPtr vert2,
                             Eigen::Vector3d ball_center)
                : vert0_(vert0),
                  vert1_(vert1),
                  vert2_(vert2),
                  ball_center_(ball_center) {}

    public:
        BallPivotingVertexPtr vert0_;
        BallPivotingVertexPtr vert1_;
        BallPivotingVertexPtr vert2_;
        Eigen::Vector3d ball_center_;
    };

    void BallPivotingVertex::UpdateType() {
        if (edges_.empty()) {
            type_ = Type::Orphan;
        } else {
            for (const BallPivotingEdgePtr &edge: edges_) {
                if (edge->type_ != BallPivotingEdge::Type::Inner) {
                    type_ = Type::Front;
                    return;
                }
            }
            type_ = Type::Inner;
        }
    }

    void BallPivotingEdge::AddAdjacentTriangle(BallPivotingTrianglePtr triangle) {
        if (triangle != triangle0_ && triangle != triangle1_) {
            if (triangle0_ == nullptr) {
                triangle0_ = triangle;
                type_ = Type::Front;
                // update orientation
                if (BallPivotingVertexPtr opp = GetOppositeVertex()) {
                    Eigen::Vector3d tr_norm =
                            (target_->point_ - source_->point_)
                                    .cross(opp->point_ - source_->point_);
                    tr_norm /= tr_norm.norm();
                    Eigen::Vector3d pt_norm =
                            source_->normal_ + target_->normal_ + opp->normal_;
                    pt_norm /= pt_norm.norm();
                    if (pt_norm.dot(tr_norm) < 0) {
                        std::swap(target_, source_);
                    }
                } else {
                }
            } else if (triangle1_ == nullptr) {
                triangle1_ = triangle;
                type_ = Type::Inner;
            } else {
            }
        }
    }

    BallPivotingVertexPtr BallPivotingEdge::GetOppositeVertex() {
        if (triangle0_ != nullptr) {
            if (triangle0_->vert0_->idx_ != source_->idx_ &&
                triangle0_->vert0_->idx_ != target_->idx_) {
                return triangle0_->vert0_;
            } else if (triangle0_->vert1_->idx_ != source_->idx_ &&
                       triangle0_->vert1_->idx_ != target_->idx_) {
                return triangle0_->vert1_;
            } else {
                return triangle0_->vert2_;
            }
        } else {
            return nullptr;
        }
    }

    class BallPivoting {
    public:
        BallPivoting(const PointCloud &pcd)
                : pcd(pcd), current_mesh(pcd) {
            for (size_t vidx = 0; vidx < pcd.points.size(); ++vidx) {
                vertices.emplace_back(new BallPivotingVertex(static_cast<int>(vidx),
                                                             pcd.points[vidx],
                                                             pcd.normals[vidx]));
            }
        }

        virtual ~BallPivoting() {
            for (auto vert: vertices) {
                delete vert;
            }
        }

        bool ComputeBallCenter(int vidx1,
                               int vidx2,
                               int vidx3,
                               double radius,
                               Eigen::Vector3d &center) {
            const Eigen::Vector3d &v1 = vertices[vidx1]->point_;
            const Eigen::Vector3d &v2 = vertices[vidx2]->point_;
            const Eigen::Vector3d &v3 = vertices[vidx3]->point_;
            double c = (v2 - v1).squaredNorm();
            double b = (v1 - v3).squaredNorm();
            double a = (v3 - v2).squaredNorm();

            double alpha = a * (b + c - a);
            double beta = b * (a + c - b);
            double gamma = c * (a + b - c);
            double abg = alpha + beta + gamma;

            if (abg < 1e-16) {
                return false;
            }

            alpha = alpha / abg;
            beta = beta / abg;
            gamma = gamma / abg;

            Eigen::Vector3d circ_center = alpha * v1 + beta * v2 + gamma * v3;
            double circ_radius2 = a * b * c;

            a = std::sqrt(a);
            b = std::sqrt(b);
            c = std::sqrt(c);
            circ_radius2 = circ_radius2 /
                           ((a + b + c) * (b + c - a) * (c + a - b) * (a + b - c));

            double height = radius * radius - circ_radius2;
            if (height >= 0.0) {
                Eigen::Vector3d tr_norm = (v2 - v1).cross(v3 - v1);
                tr_norm /= tr_norm.norm();
                Eigen::Vector3d pt_norm = vertices[vidx1]->normal_ +
                                          vertices[vidx2]->normal_ +
                                          vertices[vidx3]->normal_;
                pt_norm /= pt_norm.norm();
                if (tr_norm.dot(pt_norm) < 0) {
                    tr_norm *= -1;
                }

                height = sqrt(height);
                center = circ_center + height * tr_norm;
                return true;
            }
            return false;
        }

        BallPivotingEdgePtr GetLinkingEdge(const BallPivotingVertexPtr &v0,
                                           const BallPivotingVertexPtr &v1) {
            for (BallPivotingEdgePtr edge0: v0->edges_) {
                for (BallPivotingEdgePtr edge1: v1->edges_) {
                    if (edge0->source_->idx_ == edge1->source_->idx_ &&
                        edge0->target_->idx_ == edge1->target_->idx_) {
                        return edge0;
                    }
                }
            }
            return nullptr;
        }

        void CreateTriangle(const BallPivotingVertexPtr &v0,
                            const BallPivotingVertexPtr &v1,
                            const BallPivotingVertexPtr &v2,
                            const Eigen::Vector3d &center) {
            BallPivotingTrianglePtr triangle =
                    std::make_shared<BallPivotingTriangle>(v0, v1, v2, center);

            BallPivotingEdgePtr e0 = GetLinkingEdge(v0, v1);
            if (e0 == nullptr) {
                e0 = std::make_shared<BallPivotingEdge>(v0, v1);
            }
            e0->AddAdjacentTriangle(triangle);
            v0->edges_.insert(e0);
            v1->edges_.insert(e0);

            BallPivotingEdgePtr e1 = GetLinkingEdge(v1, v2);
            if (e1 == nullptr) {
                e1 = std::make_shared<BallPivotingEdge>(v1, v2);
            }
            e1->AddAdjacentTriangle(triangle);
            v1->edges_.insert(e1);
            v2->edges_.insert(e1);

            BallPivotingEdgePtr e2 = GetLinkingEdge(v2, v0);
            if (e2 == nullptr) {
                e2 = std::make_shared<BallPivotingEdge>(v2, v0);
            }
            e2->AddAdjacentTriangle(triangle);
            v2->edges_.insert(e2);
            v0->edges_.insert(e2);

            v0->UpdateType();
            v1->UpdateType();
            v2->UpdateType();

            Eigen::Vector3d face_normal =
                    ComputeFaceNormal(v0->point_, v1->point_, v2->point_);
            if (face_normal.dot(v0->normal_) > -1e-16) {
                current_mesh.indices.emplace_back(
                        Eigen::Vector3i(v0->idx_, v1->idx_, v2->idx_));
            } else {
                current_mesh.indices.emplace_back(
                        Eigen::Vector3i(v0->idx_, v2->idx_, v1->idx_));
            }
            current_mesh.triangle_normals.push_back(face_normal);
        }

        Eigen::Vector3d ComputeFaceNormal(const Eigen::Vector3d &v0,
                                          const Eigen::Vector3d &v1,
                                          const Eigen::Vector3d &v2) {
            Eigen::Vector3d normal = (v1 - v0).cross(v2 - v0);
            double norm = normal.norm();
            if (norm > 0) {
                normal /= norm;
            }
            return normal;
        }

        bool IsCompatible(const BallPivotingVertexPtr &v0,
                          const BallPivotingVertexPtr &v1,
                          const BallPivotingVertexPtr &v2) {
            Eigen::Vector3d normal =
                    ComputeFaceNormal(v0->point_, v1->point_, v2->point_);
            if (normal.dot(v0->normal_) < -1e-16) {
                normal *= -1;
            }
            bool ret = normal.dot(v0->normal_) > -1e-16 &&
                       normal.dot(v1->normal_) > -1e-16 &&
                       normal.dot(v2->normal_) > -1e-16;
            return ret;
        }

        BallPivotingVertexPtr FindCandidateVertex(
                const BallPivotingEdgePtr &edge,
                double radius,
                Eigen::Vector3d &candidate_center) {
            BallPivotingVertexPtr src = edge->source_;
            BallPivotingVertexPtr tgt = edge->target_;

            const BallPivotingVertex *opp = edge->GetOppositeVertex();

            Eigen::Vector3d mp = 0.5 * (src->point_ + tgt->point_);

            BallPivotingTrianglePtr triangle = edge->triangle0_;
            const Eigen::Vector3d &center = triangle->ball_center_;

            Eigen::Vector3d v = tgt->point_ - src->point_;
            v /= v.norm();

            Eigen::Vector3d a = center - mp;
            a /= a.norm();

            std::vector<int> indices;
            std::vector<double> dists2;
            pcd.kdtree->SearchRadius(mp, 2 * radius, indices, dists2);

            BallPivotingVertexPtr min_candidate = nullptr;
            double min_angle = 2 * M_PI;
            for (auto nbidx: indices) {
                const BallPivotingVertexPtr &candidate = vertices[nbidx];
                if (candidate->idx_ == src->idx_ || candidate->idx_ == tgt->idx_ ||
                    candidate->idx_ == opp->idx_) {
                    continue;
                }

                bool coplanar = points_coplanar(
                        src->point_, tgt->point_, opp->point_, candidate->point_);
                if (coplanar && (line_segments_minimum_distance(
                        mp, candidate->point_, src->point_,
                        opp->point_) < 1e-12 ||
                                 line_segments_minimum_distance(
                                         mp, candidate->point_, tgt->point_,
                                         opp->point_) < 1e-12)) {
                    continue;
                }

                Eigen::Vector3d new_center;
                if (!ComputeBallCenter(src->idx_, tgt->idx_, candidate->idx_,
                                       radius, new_center)) {
                    continue;
                }

                Eigen::Vector3d b = new_center - mp;
                b /= b.norm();

                double cosinus = a.dot(b);
                cosinus = std::min(cosinus, 1.0);
                cosinus = std::max(cosinus, -1.0);

                double angle = std::acos(cosinus);

                Eigen::Vector3d c = a.cross(b);
                if (c.dot(v) < 0) {
                    angle = 2 * M_PI - angle;
                }

                if (angle >= min_angle) {
                    continue;
                }

                bool empty_ball = true;
                for (auto nbidx2: indices) {
                    const BallPivotingVertexPtr &nb = vertices[nbidx2];
                    if (nb->idx_ == src->idx_ || nb->idx_ == tgt->idx_ ||
                        nb->idx_ == candidate->idx_) {
                        continue;
                    }
                    if ((new_center - nb->point_).norm() < radius - 1e-16) {
                        empty_ball = false;
                        break;
                    }
                }

                if (empty_ball) {
                    min_angle = angle;
                    min_candidate = vertices[nbidx];
                    candidate_center = new_center;
                }
            }
            return min_candidate;
        }

        void ExpandTriangulation(double radius) {
            while (!edge_front_deque.empty()) {
                BallPivotingEdgePtr edge = edge_front_deque.front();
                edge_front_deque.pop_front();
                if (edge->type_ != BallPivotingEdge::Front) {
                    continue;
                }

                Eigen::Vector3d center;
                BallPivotingVertexPtr candidate =
                        FindCandidateVertex(edge, radius, center);
                if (candidate == nullptr ||
                    candidate->type_ == BallPivotingVertex::Type::Inner ||
                    !IsCompatible(candidate, edge->source_, edge->target_)) {
                    edge->type_ = BallPivotingEdge::Type::Border;
                    border_edges_list.push_back(edge);
                    continue;
                }

                BallPivotingEdgePtr e0 = GetLinkingEdge(candidate, edge->source_);
                BallPivotingEdgePtr e1 = GetLinkingEdge(candidate, edge->target_);
                if ((e0 != nullptr && e0->type_ != BallPivotingEdge::Type::Front) ||
                    (e1 != nullptr && e1->type_ != BallPivotingEdge::Type::Front)) {
                    edge->type_ = BallPivotingEdge::Type::Border;
                    border_edges_list.push_back(edge);
                    continue;
                }

                CreateTriangle(edge->source_, edge->target_, candidate, center);

                e0 = GetLinkingEdge(candidate, edge->source_);
                e1 = GetLinkingEdge(candidate, edge->target_);
                if (e0->type_ == BallPivotingEdge::Type::Front) {
                    edge_front_deque.push_front(e0);
                }
                if (e1->type_ == BallPivotingEdge::Type::Front) {
                    edge_front_deque.push_front(e1);
                }
            }
        }

        bool TryTriangleSeed(const BallPivotingVertexPtr &v0,
                             const BallPivotingVertexPtr &v1,
                             const BallPivotingVertexPtr &v2,
                             const std::vector<int> &nb_indices,
                             double radius,
                             Eigen::Vector3d &center) {
            if (!IsCompatible(v0, v1, v2)) {
                return false;
            }

            BallPivotingEdgePtr e0 = GetLinkingEdge(v0, v2);
            BallPivotingEdgePtr e1 = GetLinkingEdge(v1, v2);
            if (e0 != nullptr && e0->type_ == BallPivotingEdge::Type::Inner) {
                return false;
            }
            if (e1 != nullptr && e1->type_ == BallPivotingEdge::Type::Inner) {
                return false;
            }

            if (!ComputeBallCenter(v0->idx_, v1->idx_, v2->idx_, radius, center)) {
                return false;
            }

            // test if no other point is within the ball
            for (const auto &nbidx: nb_indices) {
                const BallPivotingVertexPtr &v = vertices[nbidx];
                if (v->idx_ == v0->idx_ || v->idx_ == v1->idx_ ||
                    v->idx_ == v2->idx_) {
                    continue;
                }
                if ((center - v->point_).norm() < radius - 1e-16) {
                    return false;
                }
            }

            return true;
        }

        bool TrySeed(BallPivotingVertexPtr &v, double radius) {
            std::vector<int> indices;
            std::vector<double> dists2;
            pcd.kdtree->SearchRadius(v->point_, 2 * radius, indices, dists2);
            if (indices.size() < 3u) {
                return false;
            }

            for (size_t nbidx0 = 0; nbidx0 < indices.size(); ++nbidx0) {
                const BallPivotingVertexPtr &nb0 = vertices[indices[nbidx0]];
                if (nb0->type_ != BallPivotingVertex::Type::Orphan) {
                    continue;
                }
                if (nb0->idx_ == v->idx_) {
                    continue;
                }

                int candidate_vidx2 = -1;
                Eigen::Vector3d center;
                for (size_t nbidx1 = nbidx0 + 1; nbidx1 < indices.size();
                     ++nbidx1) {
                    const BallPivotingVertexPtr &nb1 = vertices[indices[nbidx1]];
                    if (nb1->type_ != BallPivotingVertex::Type::Orphan) {
                        continue;
                    }
                    if (nb1->idx_ == v->idx_) {
                        continue;
                    }
                    if (TryTriangleSeed(v, nb0, nb1, indices, radius, center)) {
                        candidate_vidx2 = nb1->idx_;
                        break;
                    }
                }

                if (candidate_vidx2 >= 0) {
                    const BallPivotingVertexPtr &nb1 = vertices[candidate_vidx2];

                    BallPivotingEdgePtr e0 = GetLinkingEdge(v, nb1);
                    if (e0 != nullptr &&
                        e0->type_ != BallPivotingEdge::Type::Front) {
                        continue;
                    }
                    BallPivotingEdgePtr e1 = GetLinkingEdge(nb0, nb1);
                    if (e1 != nullptr &&
                        e1->type_ != BallPivotingEdge::Type::Front) {
                        continue;
                    }
                    BallPivotingEdgePtr e2 = GetLinkingEdge(v, nb0);
                    if (e2 != nullptr &&
                        e2->type_ != BallPivotingEdge::Type::Front) {
                        continue;
                    }

                    CreateTriangle(v, nb0, nb1, center);

                    e0 = GetLinkingEdge(v, nb1);
                    e1 = GetLinkingEdge(nb0, nb1);
                    e2 = GetLinkingEdge(v, nb0);
                    if (e0->type_ == BallPivotingEdge::Type::Front) {
                        edge_front_deque.push_front(e0);
                    }
                    if (e1->type_ == BallPivotingEdge::Type::Front) {
                        edge_front_deque.push_front(e1);
                    }
                    if (e2->type_ == BallPivotingEdge::Type::Front) {
                        edge_front_deque.push_front(e2);
                    }

                    if (edge_front_deque.size() > 0) {
                        return true;
                    }
                }
            }

            return false;
        }

        void FindSeedTriangle(double radius) {
            for (auto &vertice: vertices) {
                if (vertice->type_ == BallPivotingVertex::Type::Orphan) {
                    if (TrySeed(vertice, radius)) {
                        ExpandTriangulation(radius);
                    }
                }
            }
        }

        void Run(const std::vector<double> &radii) {
            current_mesh.indices.clear();

            for (double radius: radii) {
                // update radius => update border edges
                for (auto it = border_edges_list.begin(); it != border_edges_list.end();) {
                    BallPivotingEdgePtr edge = *it;
                    BallPivotingTrianglePtr triangle = edge->triangle0_;
                    Eigen::Vector3d center;
                    if (ComputeBallCenter(triangle->vert0_->idx_,
                                          triangle->vert1_->idx_,
                                          triangle->vert2_->idx_, radius, center)) {
                        std::vector<int> indices;
                        std::vector<double> dists2;
                        pcd.kdtree->SearchRadius(center, radius, indices, dists2);
                        bool empty_ball = true;
                        for (auto idx: indices) {
                            if (idx != triangle->vert0_->idx_ &&
                                idx != triangle->vert1_->idx_ &&
                                idx != triangle->vert2_->idx_) {
                                empty_ball = false;
                                break;
                            }
                        }

                        if (empty_ball) {
                            edge->type_ = BallPivotingEdge::Type::Front;
                            edge_front_deque.push_back(edge);
                            it = border_edges_list.erase(it);
                            continue;
                        }
                    }
                    ++it;
                }

                // do the reconstruction
                if (edge_front_deque.empty()) {
                    FindSeedTriangle(radius);
                } else {
                    ExpandTriangulation(radius);
                }
            }
        }

    private:
        const PointCloud &pcd;
        std::deque<BallPivotingEdgePtr> edge_front_deque;
        std::list<BallPivotingEdgePtr> border_edges_list;
        std::vector<BallPivotingVertexPtr> vertices;
        TriangleMesh current_mesh;

    public:
        TriangleMesh ReleaseTriMesh() {
            return std::move(this->current_mesh);
        }
    };

    TriangleMesh create_triangle_mesh_ball_pivoting(
            const PointCloud &pcd, const std::vector<double> &radii) {
        BallPivoting bp(pcd);
        bp.Run(radii);
        return bp.ReleaseTriMesh();
    }

}  // namespace open3d