#include <pc2mesh/poisson/poisson.hpp>
#include <pc2mesh/geometry/poisson.hpp>

#include <Eigen/Dense>
#include <cmath>

#include <cstdlib>
#include <list>

#include <pc2mesh/geometry/point_cloud.hpp>
#include <pc2mesh/geometry/triangle_mesh.hpp>

namespace pc2mesh {
    namespace geometry {
        namespace poisson {

// The order of the B-Spline used to splat in data for color interpolation
            static const int DATA_DEGREE = 0;
// The order of the B-Spline used to splat in the weights for density estimation
            static const int WEIGHT_DEGREE = 2;
// The order of the B-Spline used to splat in the normals for constructing the
// Laplacian constraints
            static const int NORMAL_DEGREE = 2;
// The default finite-element degree
            static const int DEFAULT_FEM_DEGREE = 1;
// The default finite-element boundary type
            static const BoundaryType DEFAULT_FEM_BOUNDARY = BOUNDARY_NEUMANN;
// The dimension of the system
            static const int DIMENSION = 3;

            class Pc2MeshPoissonData {
            public:
                Pc2MeshPoissonData() : normal_(0, 0, 0) {}
                Pc2MeshPoissonData(const Eigen::Vector3d& normal)
                        : normal_(normal) {}

                Pc2MeshPoissonData operator*(double s) const {
                    return Pc2MeshPoissonData(s * normal_);
                }
                Pc2MeshPoissonData operator/(double s) const {
                    return Pc2MeshPoissonData(normal_ / s);
                }
                Pc2MeshPoissonData& operator+=(const Pc2MeshPoissonData& d) {
                    normal_ += d.normal_;
                    return *this;
                }
                Pc2MeshPoissonData& operator*=(double s) {
                    normal_ *= s;
                    return *this;
                }

            public:
                Eigen::Vector3d normal_;
            };

            template <typename Real>
            class Pc2MeshPoissonPointStream
                    : public InputPointStreamWithData<Real, DIMENSION, Pc2MeshPoissonData> {
            public:
                Pc2MeshPoissonPointStream(const PointCloud& pcd)
                        : pcd_(pcd), xform_(nullptr), current_(0) {}
                void reset(void) { current_ = 0; }
                bool nextPoint(Point<Real, 3>& p, Pc2MeshPoissonData& d) {
                    if (current_ >= pcd_.points.size()) {
                        return false;
                    }
                    p.coords[0] = static_cast<Real>(pcd_.points[current_](0));
                    p.coords[1] = static_cast<Real>(pcd_.points[current_](1));
                    p.coords[2] = static_cast<Real>(pcd_.points[current_](2));

                    if (xform_ != nullptr) {
                        p = (*xform_) * p;
                    }

                    d.normal_ = pcd_.normals[current_];

                    current_++;
                    return true;
                }

            public:
                const PointCloud& pcd_;
                XForm<Real, 4>* xform_;
                size_t current_;
            };

            template <typename RealType>
            class Pc2MeshPoissonVertex {
            public:
                using Real = RealType;

                Pc2MeshPoissonVertex() : Pc2MeshPoissonVertex(Point<Real, 3>(0, 0, 0)) {}
                Pc2MeshPoissonVertex(Point<Real, 3> point)
                        : point(point), normal_(0, 0, 0), color_(0, 0, 0), w_(0) {}

                Pc2MeshPoissonVertex& operator*=(Real s) {
                    point *= s;
                    normal_ *= s;
                    color_ *= s;
                    w_ *= s;
                    return *this;
                }

                Pc2MeshPoissonVertex& operator+=(const Pc2MeshPoissonVertex& p) {
                    point += p.point;
                    normal_ += p.normal_;
                    color_ += p.color_;
                    w_ += p.w_;
                    return *this;
                }

                Pc2MeshPoissonVertex& operator/=(Real s) {
                    point /= s;
                    normal_ /= s;
                    color_ /= s;
                    w_ /= s;
                    return *this;
                }

            public:
                // point can not have trailing _, because template methods assume that it is
                // named this way
                Point<Real, 3> point;
                Eigen::Vector3d normal_;
                Eigen::Vector3d color_;
                double w_;
            };

            template <class Real, unsigned int Dim>
            XForm<Real, Dim + 1> GetBoundingBoxXForm(Point<Real, Dim> min,
                                                     Point<Real, Dim> max,
                                                     Real scaleFactor) {
                Point<Real, Dim> center = (max + min) / 2;
                Real scale = max[0] - min[0];
                for (unsigned int d = 1; d < Dim; d++) {
                    scale = std::max<Real>(scale, max[d] - min[d]);
                }
                scale *= scaleFactor;
                for (unsigned int i = 0; i < Dim; i++) {
                    center[i] -= scale / 2;
                }
                XForm<Real, Dim + 1> tXForm = XForm<Real, Dim + 1>::Identity(),
                        sXForm = XForm<Real, Dim + 1>::Identity();
                for (unsigned int i = 0; i < Dim; i++) {
                    sXForm(i, i) = (Real)(1. / scale), tXForm(Dim, i) = -center[i];
                }
                return sXForm * tXForm;
            }

            template <class Real, unsigned int Dim>
            XForm<Real, Dim + 1> GetBoundingBoxXForm(Point<Real, Dim> min,
                                                     Point<Real, Dim> max,
                                                     Real width,
                                                     Real scaleFactor,
                                                     int& depth) {
                // Get the target resolution (along the largest dimension)
                Real resolution = (max[0] - min[0]) / width;
                for (unsigned int d = 1; d < Dim; d++) {
                    resolution = std::max<Real>(resolution, (max[d] - min[d]) / width);
                }
                resolution *= scaleFactor;
                depth = 0;
                while ((1 << depth) < resolution) {
                    depth++;
                }

                Point<Real, Dim> center = (max + min) / 2;
                Real scale = (1 << depth) * width;

                for (unsigned int i = 0; i < Dim; i++) {
                    center[i] -= scale / 2;
                }
                XForm<Real, Dim + 1> tXForm = XForm<Real, Dim + 1>::Identity(),
                        sXForm = XForm<Real, Dim + 1>::Identity();
                for (unsigned int i = 0; i < Dim; i++) {
                    sXForm(i, i) = (Real)(1. / scale), tXForm(Dim, i) = -center[i];
                }
                return sXForm * tXForm;
            }

            template <class Real, unsigned int Dim>
            XForm<Real, Dim + 1> GetPointXForm(InputPointStream<Real, Dim>& stream,
                                               Real width,
                                               Real scaleFactor,
                                               int& depth) {
                Point<Real, Dim> min, max;
                stream.boundingBox(min, max);
                return GetBoundingBoxXForm(min, max, width, scaleFactor, depth);
            }

            template <class Real, unsigned int Dim>
            XForm<Real, Dim + 1> GetPointXForm(InputPointStream<Real, Dim>& stream,
                                               Real scaleFactor) {
                Point<Real, Dim> min, max;
                stream.boundingBox(min, max);
                return GetBoundingBoxXForm(min, max, scaleFactor);
            }

            template <unsigned int Dim, typename Real>
            struct ConstraintDual {
                Real target, weight;
                ConstraintDual(Real t, Real w) : target(t), weight(w) {}
                CumulativeDerivativeValues<Real, Dim, 0> operator()(
                        const Point<Real, Dim>& p) const {
                    return CumulativeDerivativeValues<Real, Dim, 0>(target * weight);
                };
            };

            template <unsigned int Dim, typename Real>
            struct SystemDual {
                Real weight;
                SystemDual(Real w) : weight(w) {}
                CumulativeDerivativeValues<Real, Dim, 0> operator()(
                        const Point<Real, Dim>& p,
                        const CumulativeDerivativeValues<Real, Dim, 0>& dValues) const {
                    return dValues * weight;
                };
                CumulativeDerivativeValues<double, Dim, 0> operator()(
                        const Point<Real, Dim>& p,
                        const CumulativeDerivativeValues<double, Dim, 0>& dValues) const {
                    return dValues * weight;
                };
            };

            template <unsigned int Dim>
            struct SystemDual<Dim, double> {
                typedef double Real;
                Real weight;
                SystemDual(Real w) : weight(w) {}
                CumulativeDerivativeValues<Real, Dim, 0> operator()(
                        const Point<Real, Dim>& p,
                        const CumulativeDerivativeValues<Real, Dim, 0>& dValues) const {
                    return dValues * weight;
                };
            };

            template <typename Vertex,
                    typename Real,
                    typename SetVertexFunction,
                    unsigned int... FEMSigs,
                    typename... SampleData>
            void ExtractMesh(
                    float datax,
                    bool linear_fit,
                    UIntPack<FEMSigs...>,
                    std::tuple<SampleData...>,
                    FEMTree<sizeof...(FEMSigs), Real>& tree,
                    const DenseNodeData<Real, UIntPack<FEMSigs...>>& solution,
                    Real isoValue,
                    const std::vector<typename FEMTree<sizeof...(FEMSigs),
                            Real>::PointSample>* samples,
                    std::vector<Pc2MeshPoissonData>* sampleData,
                    const typename FEMTree<sizeof...(FEMSigs),
                            Real>::template DensityEstimator<WEIGHT_DEGREE>*
                    density,
                    const SetVertexFunction& SetVertex,
                    XForm<Real, sizeof...(FEMSigs) + 1> iXForm,
                    TriangleMesh& out_mesh,
                    std::vector<double>& out_densities) {
                static const int Dim = sizeof...(FEMSigs);
                typedef UIntPack<FEMSigs...> Sigs;
                static const unsigned int DataSig =
                        FEMDegreeAndBType<DATA_DEGREE, BOUNDARY_FREE>::Signature;
                typedef typename FEMTree<Dim,
                        Real>::template DensityEstimator<WEIGHT_DEGREE>
                        DensityEstimator;

                CoredMeshData<Vertex, node_index_type>* mesh;
                mesh = new CoredVectorMeshData<Vertex, node_index_type>();

                bool non_manifold = true;
                bool polygon_mesh = false;

                typename IsoSurfaceExtractor<Dim, Real, Vertex>::IsoStats isoStats;
                if (sampleData) {
                    SparseNodeData<ProjectiveData<Pc2MeshPoissonData, Real>,
                            IsotropicUIntPack<Dim, DataSig>>
                            _sampleData =
                            tree.template setMultiDepthDataField<DataSig, false>(
                                    *samples, *sampleData, (DensityEstimator*)NULL);
                    for (const RegularTreeNode<Dim, FEMTreeNodeData, depth_and_offset_type>*
                            n = tree.tree().nextNode();
                         n; n = tree.tree().nextNode(n)) {
                        ProjectiveData<Pc2MeshPoissonData, Real>* clr = _sampleData(n);
                        if (clr) (*clr) *= (Real)pow(datax, tree.depth(n));
                    }
                    isoStats = IsoSurfaceExtractor<Dim, Real, Vertex>::template Extract<
                            Pc2MeshPoissonData>(Sigs(), UIntPack<WEIGHT_DEGREE>(),
                                        UIntPack<DataSig>(), tree, density, &_sampleData,
                                        solution, isoValue, *mesh, SetVertex, !linear_fit,
                                        !non_manifold, polygon_mesh, false);
                } else {
                    isoStats = IsoSurfaceExtractor<Dim, Real, Vertex>::template Extract<
                            Pc2MeshPoissonData>(Sigs(), UIntPack<WEIGHT_DEGREE>(),
                                        UIntPack<DataSig>(), tree, density, NULL, solution,
                                        isoValue, *mesh, SetVertex, !linear_fit,
                                        !non_manifold, polygon_mesh, false);
                }

                mesh->resetIterator();
                out_densities.clear();
                for (size_t vidx = 0; vidx < mesh->outOfCorePointCount(); ++vidx) {
                    Vertex v;
                    mesh->nextOutOfCorePoint(v);
                    v.point = iXForm * v.point;
                    out_mesh.pcd.points.push_back(
                            Eigen::Vector3d(v.point[0], v.point[1], v.point[2]));
                    out_mesh.pcd.normals.push_back(v.normal_);
                    out_densities.push_back(v.w_);
                }
                for (size_t tidx = 0; tidx < mesh->polygonCount(); ++tidx) {
                    std::vector<CoredVertexIndex<node_index_type>> triangle;
                    mesh->nextPolygon(triangle);
                    out_mesh.indices.template emplace_back(triangle[0].idx, triangle[1].idx, triangle[2].idx);
                }

                delete mesh;
            }

            template <class Real, typename... SampleData, unsigned int... FEMSigs>
            void Execute(const PointCloud& pcd,
                         TriangleMesh& out_mesh,
                         std::vector<double>& out_densities,
                         int depth,
                         float width,
                         float scale,
                         bool linear_fit,
                         UIntPack<FEMSigs...>) {
                static const int Dim = sizeof...(FEMSigs);
                typedef UIntPack<FEMSigs...> Sigs;
                typedef UIntPack<FEMSignature<FEMSigs>::Degree...> Degrees;
                typedef UIntPack<FEMDegreeAndBType<
                        NORMAL_DEGREE, DerivativeBoundary<FEMSignature<FEMSigs>::BType,
                                1>::BType>::Signature...>
                        NormalSigs;
                typedef typename FEMTree<Dim,
                        Real>::template DensityEstimator<WEIGHT_DEGREE>
                        DensityEstimator;
                typedef typename FEMTree<Dim, Real>::template InterpolationInfo<Real, 0>
                        InterpolationInfo;

                XForm<Real, Dim + 1> xForm, iXForm;
                xForm = XForm<Real, Dim + 1>::Identity();

                float datax = 32.f;
                int base_depth = 0;
                int base_v_cycles = 1;
                float confidence = 0.f;
                float point_weight = 2.f * DEFAULT_FEM_DEGREE;
                float confidence_bias = 0.f;
                float samples_per_node = 1.5f;
                float cg_solver_accuracy = 1e-3f;
                int full_depth = 5;
                int iters = 8;
                bool exact_interpolation = false;

                double startTime = Time();
                Real isoValue = 0;

                FEMTree<Dim, Real> tree(MEMORY_ALLOCATOR_BLOCK_SIZE);

                size_t pointCount;

                Real pointWeightSum;
                std::vector<typename FEMTree<Dim, Real>::PointSample> samples;
                std::vector<Pc2MeshPoissonData> sampleData;
                DensityEstimator* density = NULL;
                SparseNodeData<Point<Real, Dim>, NormalSigs>* normalInfo = NULL;
                Real targetValue = (Real)0.5;

                // Read in the samples (and color data)
                {
                    Pc2MeshPoissonPointStream<Real> pointStream(pcd);

                    if (width > 0.0f) {
                        xForm = GetPointXForm<Real, Dim>(pointStream, (Real)width,
                                                         (Real)(scale > 0 ? scale : 1.),
                                                         depth) *
                                xForm;
                    } else {
                        xForm = scale > 0 ? GetPointXForm<Real, Dim>(pointStream,
                                                                     (Real)scale) *
                                            xForm
                                          : xForm;
                    }

                    pointStream.xform_ = &xForm;

                    {
                        auto ProcessDataWithConfidence = [&](const Point<Real, Dim>& p,
                                                             Pc2MeshPoissonData& d) {
                            Real l = (Real)d.normal_.norm();
                            if (!l || l != l) return (Real)-1.;
                            return (Real)pow(l, confidence);
                        };
                        auto ProcessData = [](const Point<Real, Dim>& p, Pc2MeshPoissonData& d) {
                            Real l = (Real)d.normal_.norm();
                            if (!l || l != l) return (Real)-1.;
                            d.normal_ /= l;
                            return (Real)1.;
                        };
                        if (confidence > 0) {
                            pointCount = FEMTreeInitializer<Dim, Real>::template Initialize<
                                    Pc2MeshPoissonData>(tree.spaceRoot(), pointStream, depth,
                                                samples, sampleData, true,
                                                tree.nodeAllocators[0], tree.initializer(),
                                                ProcessDataWithConfidence);
                        } else {
                            pointCount = FEMTreeInitializer<Dim, Real>::template Initialize<
                                    Pc2MeshPoissonData>(tree.spaceRoot(), pointStream, depth,
                                                samples, sampleData, true,
                                                tree.nodeAllocators[0], tree.initializer(),
                                                ProcessData);
                        }
                    }
                    iXForm = xForm.inverse();
                }

                int kernelDepth = depth - 2;

                DenseNodeData<Real, Sigs> solution;
                {
                    DenseNodeData<Real, Sigs> constraints;
                    InterpolationInfo* iInfo = NULL;
                    int solveDepth = depth;

                    tree.resetNodeIndices();

                    // Get the kernel density estimator
                    {
                        density = tree.template setDensityEstimator<WEIGHT_DEGREE>(
                                samples, kernelDepth, samples_per_node, 1);
                    }

                    // Transform the Hermite samples into a vector field
                    {
                        normalInfo = new SparseNodeData<Point<Real, Dim>, NormalSigs>();
                        std::function<bool(Pc2MeshPoissonData, Point<Real, Dim>&)>
                                ConversionFunction =
                                [](Pc2MeshPoissonData in, Point<Real, Dim>& out) {
                                    // Point<Real, Dim> n = in.template data<0>();
                                    Point<Real, Dim> n(in.normal_(0), in.normal_(1),
                                                       in.normal_(2));
                                    Real l = (Real)Length(n);
                                    // It is possible that the samples have non-zero
                                    // normals but there are two co-located samples
                                    // with negative normals...
                                    if (!l) return false;
                                    out = n / l;
                                    return true;
                                };
                        std::function<bool(Pc2MeshPoissonData, Point<Real, Dim>&, Real&)>
                                ConversionAndBiasFunction = [&](Pc2MeshPoissonData in,
                                                                Point<Real, Dim>& out,
                                                                Real& bias) {
                            // Point<Real, Dim> n = in.template data<0>();
                            Point<Real, Dim> n(in.normal_(0), in.normal_(1),
                                               in.normal_(2));
                            Real l = (Real)Length(n);
                            // It is possible that the samples have non-zero normals
                            // but there are two co-located samples with negative
                            // normals...
                            if (!l) return false;
                            out = n / l;
                            bias = (Real)(log(l) * confidence_bias /
                                          log(1 << (Dim - 1)));
                            return true;
                        };
                        if (confidence_bias > 0) {
                            *normalInfo = tree.setDataField(
                                    NormalSigs(), samples, sampleData, density,
                                    pointWeightSum, ConversionAndBiasFunction);
                        } else {
                            *normalInfo = tree.setDataField(
                                    NormalSigs(), samples, sampleData, density,
                                    pointWeightSum, ConversionFunction);
                        }
                        ThreadPool::Parallel_for(0, normalInfo->size(),
                                                 [&](unsigned int, size_t i) {
                                                     (*normalInfo)[i] *= (Real)-1.;
                                                 });
                    }

                    // Trim the tree and prepare for multigrid
                    {
                        constexpr int MAX_DEGREE = NORMAL_DEGREE > Degrees::Max()
                                                   ? NORMAL_DEGREE
                                                   : Degrees::Max();
                        tree.template finalizeForMultigrid<MAX_DEGREE>(
                                full_depth,
                                typename FEMTree<Dim, Real>::template HasNormalDataFunctor<
                                        NormalSigs>(*normalInfo),
                                normalInfo, density);
                    }

                    // Add the FEM constraints
                    {
                        constraints = tree.initDenseNodeData(Sigs());
                        typename FEMIntegrator::template Constraint<
                                Sigs, IsotropicUIntPack<Dim, 1>, NormalSigs,
                                IsotropicUIntPack<Dim, 0>, Dim>
                                F;
                        unsigned int derivatives2[Dim];
                        for (unsigned int d = 0; d < Dim; d++) derivatives2[d] = 0;
                        typedef IsotropicUIntPack<Dim, 1> Derivatives1;
                        typedef IsotropicUIntPack<Dim, 0> Derivatives2;
                        for (unsigned int d = 0; d < Dim; d++) {
                            unsigned int derivatives1[Dim];
                            for (unsigned int dd = 0; dd < Dim; dd++)
                                derivatives1[dd] = dd == d ? 1 : 0;
                            F.weights[d]
                            [TensorDerivatives<Derivatives1>::Index(derivatives1)]
                            [TensorDerivatives<Derivatives2>::Index(
                                    derivatives2)] = 1;
                        }
                        tree.addFEMConstraints(F, *normalInfo, constraints, solveDepth);
                    }

                    // Free up the normal info
                    delete normalInfo, normalInfo = NULL;

                    // Add the interpolation constraints
                    if (point_weight > 0) {
                        if (exact_interpolation) {
                            iInfo = FEMTree<Dim, Real>::
                            template InitializeExactPointInterpolationInfo<Real, 0>(
                                    tree, samples,
                                    ConstraintDual<Dim, Real>(
                                            targetValue,
                                            (Real)point_weight * pointWeightSum),
                                    SystemDual<Dim, Real>((Real)point_weight *
                                                          pointWeightSum),
                                    true, false);
                        } else {
                            iInfo = FEMTree<Dim, Real>::
                            template InitializeApproximatePointInterpolationInfo<
                                    Real, 0>(
                                    tree, samples,
                                    ConstraintDual<Dim, Real>(
                                            targetValue,
                                            (Real)point_weight * pointWeightSum),
                                    SystemDual<Dim, Real>((Real)point_weight *
                                                          pointWeightSum),
                                    true, 1);
                        }
                        tree.addInterpolationConstraints(constraints, solveDepth, *iInfo);
                    }

                    // Solve the linear system
                    {
                        typename FEMTree<Dim, Real>::SolverInfo sInfo;
                        sInfo.cgDepth = 0, sInfo.cascadic = true, sInfo.vCycles = 1,
                        sInfo.iters = iters, sInfo.cgAccuracy = cg_solver_accuracy,
                        sInfo.verbose = false,
                        sInfo.showResidual = false,
                        sInfo.showGlobalResidual = SHOW_GLOBAL_RESIDUAL_NONE,
                        sInfo.sliceBlockSize = 1;
                        sInfo.baseDepth = base_depth, sInfo.baseVCycles = base_v_cycles;
                        typename FEMIntegrator::template System<Sigs,
                                IsotropicUIntPack<Dim, 1>>
                                F({0., 1.});
                        solution = tree.solveSystem(Sigs(), F, constraints, solveDepth,
                                                    sInfo, iInfo);
                        if (iInfo) delete iInfo, iInfo = NULL;
                    }
                }

                {
                    double valueSum = 0, weightSum = 0;
                    typename FEMTree<Dim, Real>::template MultiThreadedEvaluator<Sigs, 0>
                            evaluator(&tree, solution);
                    std::vector<double> valueSums(ThreadPool::NumThreads(), 0),
                            weightSums(ThreadPool::NumThreads(), 0);
                    ThreadPool::Parallel_for(
                            0, samples.size(), [&](unsigned int thread, size_t j) {
                                ProjectiveData<Point<Real, Dim>, Real>& sample =
                                        samples[j].sample;
                                Real w = sample.weight;
                                if (w > 0)
                                    weightSums[thread] += w,
                                            valueSums[thread] +=
                                                    evaluator.values(sample.data / sample.weight,
                                                                     thread, samples[j].node)[0] *
                                                    w;
                            });
                    for (size_t t = 0; t < valueSums.size(); t++)
                        valueSum += valueSums[t], weightSum += weightSums[t];
                    isoValue = (Real)(valueSum / weightSum);
                }

                auto SetVertex = [](Pc2MeshPoissonVertex<Real>& v, Point<Real, Dim> p, Real w,
                                    Pc2MeshPoissonData d) {
                    v.point = p;
                    v.normal_ = d.normal_;
                    v.w_ = w;
                };
                ExtractMesh<Pc2MeshPoissonVertex<Real>, Real>(
                        datax, linear_fit, UIntPack<FEMSigs...>(),
                        std::tuple<SampleData...>(), tree, solution, isoValue, &samples,
                        &sampleData, density, SetVertex, iXForm, out_mesh, out_densities);

                if (density) delete density, density = nullptr;
            }

        }  // namespace poisson

        std::pair<TriangleMesh, std::vector<double>>
        create_triangle_mesh_possion(PointCloud& pcd,
                                                  size_t depth,
                                                  float width,
                                                  float scale,
                                                  bool linear_fit,
                                                  int n_threads) {
            static const BoundaryType BType = poisson::DEFAULT_FEM_BOUNDARY;
            typedef IsotropicUIntPack<
                    poisson::DIMENSION,
                    FEMDegreeAndBType</* Degree */ 1, BType>::Signature>
                    FEMSigs;
            if (n_threads <= 0) {
                n_threads = (int)std::thread::hardware_concurrency();
            }

            ThreadPool::Init((ThreadPool::ParallelType)(int)ThreadPool::OPEN_MP,
                             n_threads);

            auto mesh = TriangleMesh { pcd };
            std::vector<double> densities;
            poisson::Execute<float>(pcd, mesh, densities, static_cast<int>(depth),
                                    width, scale, linear_fit, FEMSigs());

            ThreadPool::Terminate();

            return {std::move(mesh), densities};
        }

    }  // namespace geometry
}  // namespace open3d