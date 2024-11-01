#pragma once

#include "types.h"
#include <memory>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include "ankerl/unordered_dense.h"

#include "phtree-cpp/include/phtree/phtree.h"
#include "phtree-cpp/include/phtree/filter.h"

#include <ceres/ceres.h>


typedef std::tuple<int, int, int> GridIndex;

typedef improbable::phtree::PhPointD<3> PointPh;
template <typename V>
using TreePh = improbable::phtree::PhTreeD<3, V>;

typedef improbable::phtree::DistanceEuclidean<3> DistancePh;

template <typename V>
using HashMap = ankerl::unordered_dense::map<GridIndex, V>;

struct GPCellHyperparameters {
    int ttl;
    double lengthscale;
    double inv_lengthscale2;
    double sz2;
    double two_beta_l_2;
    double inv_2_beta_l_2;
    double beta;
    double inv_beta;

    GPCellHyperparameters(const double lengthscale, const double beta, const double sz, const int ttl);
};

class MapDistField;

class Cell {
    private:
        const GPCellHyperparameters& hyperparameters_;
        GridIndex index_;
        VecX alpha_;
        Vec3 sum_;
        Vec3 color_sum_;
        Vec3 dir_sum_;
        int count_;
        int last_alpha_update = -1;
        MatX neighbor_pts_;
        std::mutex mutex_;
        const int& global_counter_;
        double first_time_ = -1.0;
        MapDistField* map_;

        bool use_weights_ = true;
        int max_count_ = 0;

        MatX computeAlpha();

        MatX getNeighborPts(bool with_count=false);

        VecX getWeights(const MatX& pts) const;

    public:
        Cell(GridIndex index, Vec3 pt, const GPCellHyperparameters& hyperparameters, const int& global_counter=0, const double first_time=-1.0, const Vec3& pos=Vec3::Zero(), MapDistField* map=nullptr);

        MatX kernelRQ(const MatX& X1, const MatX& X2) const;
        std::tuple<MatX, MatX, MatX, MatX> kernelRQAndDiff(const MatX& X1, const MatX& X2);
        

        double revertingRQ(const double& x) const;
        std::pair<double, double> revertingRQAndDiff(const double& x) const;

        void getNeighbors(std::unordered_set<Cell*>& neighbors);
        GridIndex getIndex() const;


        void resetAlpha();
        void addPt(const Vec3& pt, const Vec3& pos=Vec3::Zero());
        void addColor(const unsigned char r, const unsigned char g, const unsigned char b);

        Vec3 getPt() const;
        std::tuple<unsigned char, unsigned char, unsigned char> getColor() const;

        bool getSign(const Vec3& pt);

        double getDist(const Vec3& pt);
        std::pair<double, Vec3> getDistAndGrad(const Vec3& pt);

        double getSdf(const Vec3& pt);
        Vec3 getSdfGrad(const Vec3& pt);

        int getCount() const { return count_; }

        double getAvgTime();

        double getFirstTime() const { return first_time_; }

        void prepareErase();
        void removeNeighbor(Cell* neighbor);

        bool hasNeighbors() const { return true; } //TODO

        Vec3 getDir() const { return dir_sum_.normalized(); }

        std::vector<Vec3> getNormals(const std::vector<Vec3>& pts, bool orientate=true, bool clean_behind=false);


        void setNoiseModel(const bool use_independent_noise, const int max_count)
        {
            use_weights_ = use_independent_noise;
            max_count_ = max_count;
        }
};

typedef std::shared_ptr<Cell> CellPtr;

inline void testCell()
{
    const double quantum = 1e-5;
    const GPCellHyperparameters hyperparameters(0.52, 32.2, 0.08, 2);
    Cell cell(GridIndex{0, 0, 0}, Vec3(0.0, 0.0, 0.0), hyperparameters);
    double occ = Vec3::Random()[0];
    auto [dist, grad] = cell.revertingRQAndDiff(occ);
    double grad_num = (cell.revertingRQ(occ + quantum) - cell.revertingRQ(occ)) / quantum;
    std::cout << "Grad diff: " << (grad - grad_num) << std::endl;

    MatX X1 = MatX::Random(2, 3);
    MatX X2 = MatX::Random(10, 3);
    auto [K, diff_K1, diff_K2, diff_K3] = cell.kernelRQAndDiff(X1, X2);
    MatX diff_K1_num = MatX::Zero(2, 10);
    MatX diff_K2_num = MatX::Zero(2, 10);
    MatX diff_K3_num = MatX::Zero(2, 10);
    for(int i = 0; i < 2; i++)
    {
        X1(i, 0) += quantum;
        MatX K_plus = cell.kernelRQ(X1, X2);
        X1(i, 0) -= quantum;
        diff_K1_num.row(i) = (K_plus.row(i) - K.row(i)) / quantum;

        X1(i, 1) += quantum;
        MatX K_plus2 = cell.kernelRQ(X1, X2);
        X1(i, 1) -= quantum;
        diff_K2_num.row(i) = (K_plus2.row(i) - K.row(i)) / quantum;

        X1(i, 2) += quantum;
        MatX K_plus3 = cell.kernelRQ(X1, X2);
        X1(i, 2) -= quantum;
        diff_K3_num.row(i) = (K_plus3.row(i) - K.row(i)) / quantum;
    }
    std::cout << "Diff K1 norm: " << (diff_K1 - diff_K1_num).norm() << std::endl;
    std::cout << "Diff K2 norm: " << (diff_K2 - diff_K2_num).norm() << std::endl;
    std::cout << "Diff K3 norm: " << (diff_K3 - diff_K3_num).norm() << std::endl;
}


struct MapDistFieldOptions {
    double cell_size = 0.15;
    int neighborhood_size = 2;
    bool use_temporal_weights = true;
    bool free_space_carving = true;
    double free_space_carving_radius = -1.0;
    double min_range = 1.0;
    double max_range = 1000.0;
    bool output_normals = false;
    bool output_mesh = false;
    double clean_mesh_threshold = 0.0;
    double meshing_point_per_node = 2.0;
};
    



class MapDistField {
    private:
        int num_neighbors_ = 2;

        std::unique_ptr<HashMap<CellPtr> > hash_map_;
        GPCellHyperparameters hyperparameters_;
        const double cell_size_;
        const double inv_cell_size_;
        const float cell_size_f_;
        const float half_cell_size_f_;
        const int dim_ = 3;


        Vec3 min_bounds_ = Vec3(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
        Vec3 max_bounds_ = Vec3(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());

        TreePh<CellPtr> phtree_;


        size_t num_cells_ = 0;

        std::tuple<std::vector<Vec3>, std::set<std::array<int, 2> > > getPointsAndEdges();

        void updateBounds(const Vec3& pt);

        int scan_counter_ = -1;

        bool has_color_ = false;

        MapDistFieldOptions opt_;

    public:
        MapDistField(const MapDistFieldOptions& options);


        Mat4 registerPts(const std::vector<Vec3>& pts, const Mat4& prior, const double current_time, const bool approximate=false, bool use_loss=true);

        void addPts(const std::vector<Pointd>& pts, const Mat4& pose);
        std::vector<Pointf> getPts();
        std::pair<std::vector<Pointf>, std::vector<Vec3> > getPtsAndNormals(bool clean_behind=false);
        std::vector<double> queryDistField(const std::vector<Vec3>& query_pts, const bool field=true);
        std::vector<double> querySdf(const std::vector<Vec3>& query_pts);
        std::pair<double, Vec3> queryDistFieldAndGrad(const Vec3& query_pts, const bool field=true);
        double queryDistField(const Vec3& query_pt, const bool field=true);
        double querySdf(const Vec3& query_pt);

        double getAvgTime(const Vec3& pt);

        void display(const double inf_resolution = 0.0);

        GridIndex getGridIndex(const Vec3& pt);
        GridIndex getGridIndex(const Vec2& pt);
        Vec3 getCenterPt(const GridIndex& index);
        std::pair<Vec3, Vec3> getBounds() const { return std::make_pair(min_bounds_, max_bounds_);}
        void setSz(const double sz) { hyperparameters_.sz2 = sz*sz; }


        void writeMap(const std::string& filename);

        bool isInHash(const GridIndex& index) const{ return hash_map_->find(index) != hash_map_->end(); }

        double distToClosestCell(const Vec3& pt) const;

        std::vector<CellPtr> getNeighborCells(const Vec3& pt);
};

//inline void testMapDistField()
//{
//    const int N_map = 100;
//    const int N_query = 10;
//    MapDistField map(1);
//    std::vector<Vec3> pts;
//    for(int i = 0; i < N_map; i++)
//    {
//        pts.push_back(Vec3(10.0*rand()/RAND_MAX, 10.0*rand()/RAND_MAX, 10.0*rand()/RAND_MAX));
//    }
//    map.addPts(pts);
//
//    const double quantum = 1e-5;
//    for(int i = 0; i < N_query; i++)
//    {
//        Vec3 query_pt(10.0*rand()/RAND_MAX, 10.0*rand()/RAND_MAX, 10.0*rand()/RAND_MAX);
//        auto [dist, grad] = map.queryDistFieldAndGrad(query_pt);
//        Vec3 grad_num;
//        for(int j = 0; j < 3; j++)
//        {
//            Vec3 query_pt_plus = query_pt;
//            query_pt_plus[j] += quantum;
//            double dist_plus = map.queryDistField(query_pt_plus);
//            grad_num[j] = (dist_plus - dist) / quantum;
//        }
//        std::cout << "Grad diff: " << (grad - grad_num).norm() << std::endl;
//    }
//}


class RegistrationCostFunction: public ceres::CostFunction
{
    public:
        RegistrationCostFunction(const std::vector<Vec3>& pts, const Mat4& prior, MapDistField* map, const std::vector<double>& weights, const double cauchy_loss_scale=0.2, const bool use_field=true, const bool use_loss=true);
        
        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

        void setUseField(const bool use_field);

    private:
        const std::vector<Vec3>& pts_;
        const Mat4 prior_;
        MapDistField* map_;
        const std::vector<double>& weights_;
        bool use_field_;

        std::unique_ptr<ceres::LossFunction> loss_function_;


};