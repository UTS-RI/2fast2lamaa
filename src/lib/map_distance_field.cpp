#include "lice/map_distance_field.h"
#include "lice/utils.h"
#include "lice/math_utils.h"
#include "happly/happly.h"

#include "PoissonRecon/Src/PoissonReconWrapped.h"

#include <iostream>
#include <eigen3/Eigen/Dense>


struct PhNeighborQuery
{
    void operator()(const PointPh& point, CellPtr& cell)
    {
        neighbors.push_back({point, cell});
    }
    std::vector<std::pair<PointPh, CellPtr> > neighbors;
};





// Function to solve the linear system with Eigen's cholesky decomposition
inline VecX solveKinvY(const MatX& K, const VecX& Y)
{
    MatX L(K.rows(), K.cols());
    Eigen::LLT<Eigen::MatrixXd> lltOfA(K);
    L = lltOfA.matrixL();
    VecX alpha;
    alpha = L.triangularView<Eigen::Lower>().solve(Y);
    L.triangularView<Eigen::Lower>().transpose().solveInPlace(alpha);
    return alpha;
}


GPCellHyperparameters::GPCellHyperparameters(const double _lengthscale, const double _beta, const double _sz, const int _ttl)
{
    lengthscale = _lengthscale;
    inv_lengthscale2 = 1.0/(_lengthscale*_lengthscale);

    sz2 = _sz*_sz;
    two_beta_l_2 = 2.0*_beta*_lengthscale*_lengthscale;
    inv_2_beta_l_2 = 1.0/two_beta_l_2;
    beta = _beta;
    inv_beta = 1.0/_beta;
    ttl = _ttl;
}



Cell::Cell(GridIndex index, Vec3 pt, const GPCellHyperparameters& hyperparameters, const int& global_counter, const double t, const Vec3& pos, MapDistField* map_dist_field):
    hyperparameters_(hyperparameters)
    , index_(index)
    , sum_(pt)
    , color_sum_(Vec3::Zero())
    , dir_sum_((pos-pt).normalized())
    , count_(1)
    , global_counter_(global_counter)
    , first_time_(t)
    , map_(map_dist_field)
{
    alpha_.resize(0);
}

void Cell::addPt(const Vec3& pt, const Vec3& pos)

{
    sum_ += pt;
    dir_sum_ += (pos-pt).normalized();
    count_++;
}
void Cell::addColor(const unsigned char r, const unsigned char g, const unsigned char b)
{
    color_sum_ += Vec3(r, g, b);
}

Vec3 Cell::getPt() const
{
    return sum_/count_;
}

std::tuple<unsigned char, unsigned char, unsigned char> Cell::getColor() const
{
    return std::make_tuple(color_sum_[0]/count_, color_sum_[1]/count_, color_sum_[2]/count_);
}

GridIndex Cell::getIndex() const
{
    return index_;
}



void Cell::resetAlpha()
{
    alpha_.resize(0);
    last_alpha_update = -2;
    neighbor_pts_ = MatX(0, 0);
}

void Cell::getNeighbors(std::unordered_set<Cell*>& neighbors)
{
    std::vector<CellPtr> neighbors_vec = map_->getNeighborCells(getPt());
    for(auto& neighbor : neighbors_vec)
    {
        neighbors.insert(neighbor.get());
    }
}


VecX Cell::getWeights(const MatX& pts) const
{
    if (!use_weights_)
    {
        return VecX::Ones(pts.rows());
    }
    double max_count = (max_count_ > 0) ? max_count_ : pts.col(3).maxCoeff();
    VecX weights = (((1.0 + hyperparameters_.sz2) * (1.0+((12.0*(pts.col(3)/max_count)).array()-6.0).exp()).inverse().matrix()).array() + hyperparameters_.sz2).matrix();
    return weights;
}


MatX Cell::computeAlpha(bool clean_behind)
{
    MatX pts = getNeighborPts(true);
    MatX weights = getWeights(pts).asDiagonal();
    double max_count = pts.col(3).maxCoeff();
    normal_weight_ = std::max(1.0,1.0-(1.0 / (1.0+std::exp(12.0*(count_/max_count)-6.0)))+0.005);
    MatX K = kernelRQ(pts.block(0,0,pts.rows(),3), pts.block(0,0,pts.rows(),3)) + weights;
    VecX Y = VecX::Ones(pts.rows());
    alpha_.resize(pts.rows());
    alpha_ = solveKinvY(K, Y);
    last_alpha_update = global_counter_;
    if(!clean_behind)
    {
        map_->cellToClean(this);
    }
    return pts.block(0, 0, pts.rows(), 3);
}

MatX Cell::getNeighborPts(bool with_count)
{
    std::unordered_set<Cell*> neighbors;
    getNeighbors(neighbors);
    MatX pts(neighbors.size(), (with_count ? 4 : 3));
    int i = 0;
    for(auto& neighbor : neighbors)
    {
        pts.block(i, 0, 1, 3) = neighbor->getPt().transpose();
        if (with_count)
        {
            pts(i, 3) = neighbor->getCount();
        }
        i++;
    }
    return pts;
}

double Cell::getAvgTime()
{
    if(first_time_ < 0)
    {
        return -1;
    }
    std::unordered_set<Cell*> neighbors;
    getNeighbors(neighbors);
    double sum = 0;
    for(auto& neighbor : neighbors)
    {
        sum += neighbor->getFirstTime();
    }
    return sum/neighbors.size();
}





MatX Cell::kernelRQ(const MatX& X1, const MatX& X2) const
{
    MatX K(X1.rows(), X2.rows());
    for(int i = 0; i < X1.rows(); i++)
    {
        for(int j = 0; j < X2.rows(); j++)
        {
            K(i, j) = std::pow((1.0+((X1.row(i) - X2.row(j)).squaredNorm()*hyperparameters_.inv_2_beta_l_2)), -hyperparameters_.beta);
        }
    }
    return K;
}



std::tuple<MatX, MatX, MatX, MatX> Cell::kernelRQAndDiff(const MatX& X1, const MatX& X2)
{
    MatX K(X1.rows(), X2.rows());
    MatX K_diff_1(X1.rows(), X2.rows());
    MatX K_diff_2(X1.rows(), X2.rows());
    MatX K_diff_3(X1.rows(), X2.rows());
    for(int i = 0; i < X1.rows(); i++)
    {
        for(int j = 0; j < X2.rows(); j++)
        {
            double dist2 = (X1.row(i) - X2.row(j)).squaredNorm();
            Vec3 diff = X2.row(j) - X1.row(i);
            double temp = 1.0+(dist2*hyperparameters_.inv_2_beta_l_2);
            K(i, j) = std::pow(temp, -hyperparameters_.beta);
            temp = std::pow(temp, -hyperparameters_.beta-1)*hyperparameters_.inv_lengthscale2;
            K_diff_1(i, j) = diff[0]*temp;
            K_diff_2(i, j) = diff[1]*temp;
            K_diff_3(i, j) = diff[2]*temp;
        }
    }
    return {K, K_diff_1, K_diff_2, K_diff_3};
}


double Cell::revertingRQ(const double& occ) const
{
    if(occ <= 0)
    {
        return -1;
    }
    else if (occ >= 1)
    {
        return 0;
    }
    else
    {
        return std::sqrt((std::pow(occ, -hyperparameters_.inv_beta) - 1) * hyperparameters_.two_beta_l_2);
    }
}

std::pair<double, double> Cell::revertingRQAndDiff(const double& occ) const
{
    if(occ <= 0)
    {
        return {1000, 0};
    }
    else if (occ >= 1)
    {
        return {0, 0};
    }
    else
    {
        double temp = (std::pow(occ, -hyperparameters_.inv_beta) - 1) * hyperparameters_.two_beta_l_2;
        double dist = std::sqrt(temp);
        double d_dist_d_occ = -hyperparameters_.two_beta_l_2*std::pow(occ, -hyperparameters_.inv_beta-1)*hyperparameters_.inv_beta/(2.0*dist);

        return {dist, d_dist_d_occ};
    }
}




double Cell::getDist(const Vec3& pt)
{
    mutex_.lock();
    if(global_counter_ != last_alpha_update)
    {
        neighbor_pts_ = computeAlpha();
    }
    mutex_.unlock();
    MatX k = kernelRQ(pt.transpose(), neighbor_pts_);

    double occ = (k*alpha_)[0];
    if (occ < 0)
    {
        return (pt - getPt()).norm();
    }
    double dist = revertingRQ(occ);
    if (dist < 0)
    {
        return (pt - getPt()).norm();
    }
    return dist;
}

std::pair<double, Vec3> Cell::getDistAndGrad(const Vec3& pt)
{
    mutex_.lock();
    if(global_counter_ != last_alpha_update)
    {
        neighbor_pts_ = computeAlpha();
    }
    mutex_.unlock();
    auto [k, k_diff_1, k_diff_2, k_diff_3] = kernelRQAndDiff(pt.transpose(), neighbor_pts_);

    double occ = (k*alpha_)[0];
    Vec3 occ_grad;
    occ_grad[0] = (k_diff_1*alpha_)[0];
    occ_grad[1] = (k_diff_2*alpha_)[0];
    occ_grad[2] = (k_diff_3*alpha_)[0];
    if (occ <= 0)
    {
        Vec3 temp_vec = pt - getPt();
        double temp_dist = (temp_vec).norm();
        Vec3 temp_grad = temp_vec / temp_dist;
        return {temp_dist, temp_grad};
    }
    //double dist = revertingRQ(occ);
    auto[dist, d_dist_d_occ] = revertingRQAndDiff(occ);
    if (dist < 0)
    {
        Vec3 temp_vec = pt - getPt();
        double temp_dist = (temp_vec).norm();
        Vec3 temp_grad = temp_vec / temp_dist;
        return {temp_dist, temp_grad};
    }
    return {dist, d_dist_d_occ*occ_grad};
}

double Cell::getSdf(const Vec3& pt)
{
    Vec3 dir = (dir_sum_/count_).normalized();
    auto [dist, grad] = getDistAndGrad(pt);
    double same_dir = dir.dot(grad);
    if(same_dir < 0)
    {
        return -dist;
    }
    return dist;
}

bool Cell::getSign(const Vec3& pt)
{   
    Vec3 occ_grad = getNormals({pt}, false)[0];

    return occ_grad.dot(dir_sum_) > 0;    
}


std::vector<Vec3> Cell::getNormals(const std::vector<Vec3>& pts, bool orientate, bool clean_behind, bool weighted)
{
    std::vector<Vec3> normals(pts.size());
    for(size_t i = 0; i < pts.size(); i++)
    {
        Vec3 occ_grad;
        {
            mutex_.lock();
            if(global_counter_ != last_alpha_update)
            {
                neighbor_pts_ = computeAlpha(clean_behind);
            }
            mutex_.unlock();
            auto [k, k_diff_1, k_diff_2, k_diff_3] = kernelRQAndDiff(pts[i].transpose(), neighbor_pts_);

            occ_grad[0] = (k_diff_1*alpha_)[0];
            occ_grad[1] = (k_diff_2*alpha_)[0];
            occ_grad[2] = (k_diff_3*alpha_)[0];
        }

        normals[i] = occ_grad.normalized();

        // Flip the normal if it is pointing in the wrong direction
        if(orientate && occ_grad.dot(dir_sum_) < 0)
        {
            normals[i] = -occ_grad;
        }
        if(clean_behind)
        {
            resetAlpha();
        }
        if(weighted)
        {
            normals[i] *= normal_weight_;
        }
    }
    return normals;
}
        





Vec3 Cell::getSdfGrad(const Vec3& pt)
{
    Vec3 dir = (dir_sum_/count_).normalized();
    auto [dist, grad] = getDistAndGrad(pt);
    double same_dir = dir.dot(grad);
    if(same_dir < 0)
    {
        return -grad;
    }
    return grad;
}


MapDistField::MapDistField(const MapDistFieldOptions& options):
    hyperparameters_(3.0*options.cell_size, 50.0, 0.1, options.neighborhood_size-1)
    , cell_size_(options.cell_size)
    , inv_cell_size_(1.0/options.cell_size)
    , cell_size_f_((float)options.cell_size)
    , half_cell_size_f_((float)options.cell_size/2.0)
    , opt_(options)
{
}


void MapDistField::updateBounds(const Vec3& pt)
{
    for(int i = 0; i < 3; i++)
    {
        min_bounds_[i] = std::min(min_bounds_[i], pt[i]);
        max_bounds_[i] = std::max(max_bounds_[i], pt[i]);
    }
}




Mat4 MapDistField::registerPts(const std::vector<Vec3>& pts, const Mat4& pose, const double current_time, const bool approximate, bool use_loss)
{
    std::cout << "Registering points" << std::endl;
    Vec6 pose_correction_state = Vec6::Zero();
    
    int num_neighbors_save = num_neighbors_;
    num_neighbors_ = 3;

    ceres::Problem problem;

    StopWatch sw;
    sw.start();
    std::vector<double> weights(pts.size(), 1.0);
    if(opt_.use_temporal_weights)
    {
        #pragma omp parallel for num_threads(16)
        for(size_t i = 0; i < pts.size(); i++)
        {
            weights[i] = getAvgTime(pts[i]);
        }
        double min_time = std::numeric_limits<double>::max();
        for(size_t i = 0; i < pts.size(); i++)
        {
            min_time = std::min(min_time, weights[i]);
        }
        double coeff = -9.0/(current_time - min_time);
        for(size_t i = 0; i < pts.size(); i++)
        {
            weights[i] = 10.0 + (weights[i] - min_time)*coeff;
        }
    }
    sw.stop();
    sw.print("Time to compute weights");

    auto temp_pose = pose;

    RegistrationCostFunction* cost_function = new RegistrationCostFunction(pts, temp_pose, this, weights, 1.5*cell_size_, false, use_loss);
    problem.AddResidualBlock(cost_function, NULL, pose_correction_state.data());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.max_num_iterations = approximate ? 15 : 10;
    options.function_tolerance = 1e-4;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;


    if(!approximate)
    {
        cost_function->setUseField(true);

        options.max_num_iterations = 2;
        options.function_tolerance = 1e-4;
        options.minimizer_progress_to_stdout = false;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;
    }


    Mat3 R = expMap(pose_correction_state.segment<3>(3));
    Vec3 pos = pose_correction_state.segment<3>(0);

    Mat4 pose_correction = Mat4::Identity();
    pose_correction.block<3,3>(0,0) = R;
    pose_correction.block<3,1>(0,3) = pos;

    num_neighbors_ = num_neighbors_save;

    return temp_pose*pose_correction;
}




void MapDistField::cellToClean(Cell* cell)
{
    clean_mutex_.lock();
    cells_to_clean_.insert(cell);
    clean_mutex_.unlock();
}

void MapDistField::cleanCells()
{
    clean_mutex_.lock();
    for(auto& cell : cells_to_clean_)
    {
        cell->resetAlpha();
    }
    cells_to_clean_.clear();
    clean_mutex_.unlock();
}



void MapDistField::addPts(const std::vector<Pointd>& pts, const Mat4& pose)
{
    cleanCells();
    if (pts.size() == 0)
    {
        return;
    }
    if(scan_counter_ < 0)
    {
        has_color_ = pts[0].has_color;
    }

    scan_counter_++;
    StopWatch sw;

    if(opt_.free_space_carving)
    {
        sw.start();

        // Create an image like structure to store the current points
        std::vector<std::tuple<int, int, double> > raw_indices(pts.size());
        const double ang_res = 0.1;
        const double ang_res_inv = 1.0/ang_res;
        int min_el_idx = std::numeric_limits<int>::max();
        int max_el_idx = std::numeric_limits<int>::lowest();
        int min_az_idx = std::numeric_limits<int>::max();
        int max_az_idx = std::numeric_limits<int>::lowest();
        std::vector<bool> valid_pts(pts.size(), true);
        int valid_count = 0;
        for(size_t i = 0; i < pts.size(); i++)
        {
            if(!std::isfinite(pts[i].x) || !std::isfinite(pts[i].y) || !std::isfinite(pts[i].z))
            {
                valid_pts[i] = false;
                continue;
            }
            float range = pts[i].vec3f().norm();
            if( (range < opt_.min_range) || (range > opt_.max_range)) 
            {
                valid_pts[i] = false;
                continue;
            }
            Vec3 polar = toPolar(pts[i].vec3());
            int el_idx = std::floor(polar[1]*ang_res_inv);
            int az_idx = std::floor(polar[2]*ang_res_inv);
            min_el_idx = std::min(min_el_idx, el_idx);
            max_el_idx = std::max(max_el_idx, el_idx);
            min_az_idx = std::min(min_az_idx, az_idx);
            max_az_idx = std::max(max_az_idx, az_idx);
            raw_indices[i] = {el_idx, az_idx, polar[0]};
            valid_count++;
        }
        if (valid_count > 0)
        {
            
            int nb_el = max_el_idx - min_el_idx + 1;
            int nb_az = max_az_idx - min_az_idx + 1;
            std::vector<std::vector<double> > dists(nb_el, std::vector<double>(nb_az, -1));
            for(size_t i = 0; i < pts.size(); i++)
            {
                if(!valid_pts[i]) continue;
                auto [el_idx, az_idx, r] = raw_indices[i];
                if((el_idx - min_el_idx < 1) || (el_idx - min_el_idx >= nb_el-1)) continue;
                double cell_dist = dists[el_idx-min_el_idx][az_idx-min_az_idx];
                if(cell_dist < 0)
                {
                    dists[el_idx-min_el_idx][az_idx-min_az_idx] = r;
                }
                else
                {
                    dists[el_idx-min_el_idx][az_idx-min_az_idx] = std::min(cell_dist, r);
                }
            }
            std::vector<std::vector<double> > dists_copy = dists;
            for(int el = 0; el < nb_el; el++)
            {
                for(int az = 0; az < nb_az; az++)
                {
                    double min_dist = std::numeric_limits<double>::max();
                    int count = 0;
                    for(int i = std::max(0, el-1); i <= std::min(nb_el-1, el+1); i++)
                    {
                        for(int j = std::max(0, az-1); j <= std::min(nb_az-1, az+1); j++)
                        {
                            min_dist = std::min(min_dist, dists_copy[i][j]);
                            count++;
                        }
                    }
                    if(count > 0)
                    {
                        dists[el][az] = min_dist;
                    }
                    else
                    {
                        dists[el][az] = -1;
                    }
                }
            }



            // Query the map points that are in the carving radius
            PhNeighborQuery query;
            phtree_.for_each(query, improbable::phtree::FilterSphere({pose(0,3), pose(1,3), pose(2,3)}, opt_.free_space_carving_radius, phtree_.converter()));

            auto& neighbors = query.neighbors;

            
            // Transform the map points to the scan frame and get the indices in the image like structure
            Mat4 pose_inv = pose.inverse();
            std::vector<std::pair<PointPh, CellPtr> > map_pts_to_remove;
            map_pts_to_remove.reserve(neighbors.size());
            double dist_threshold = 1.0*cell_size_;
            double min_range_threshold = opt_.min_range + dist_threshold;
            for(auto& neighbor : neighbors)
            {
                Vec3 temp_pt = pose_inv.block<3,3>(0,0)*neighbor.second->getPt() + pose_inv.block<3,1>(0,3);
                Vec3 polar = toPolar(temp_pt);
                int el_idx = std::floor(polar[1]*ang_res_inv) - min_el_idx;
                int az_idx = std::floor(polar[2]*ang_res_inv) - min_az_idx;
                if((el_idx < 0) || (el_idx >= nb_el)) continue;
                if((az_idx < 0) || (az_idx >= nb_az)) continue;

                if((dists[el_idx][az_idx] > 0) && (polar[0] > min_range_threshold) && (dists[el_idx][az_idx]-dist_threshold > polar[0]))
                {
                    map_pts_to_remove.push_back(neighbor);
                }
            }

            std::cout << "Number of points to remove (free space carving): " << map_pts_to_remove.size() << std::endl;


            // Remove the points from the map
            for(auto& map_pt : map_pts_to_remove)
            {
                // For each cell to remove, also remove the cell from the neighbors
                PhNeighborQuery query_to_remove;
                phtree_.for_each(query_to_remove, improbable::phtree::FilterSphere({map_pt.first[0], map_pt.first[1], map_pt.first[2]}, 2*cell_size_, phtree_.converter()));
                for(auto& neighbor : query_to_remove.neighbors)
                {
                    phtree_.erase(neighbor.first);
                    hash_map_->erase(neighbor.second->getIndex());
                    num_cells_--;
                }
            }


            sw.stop();
            sw.print("Free space carving time");
        }
    }


    sw.reset();
    sw.start();
    if(!hash_map_)
    {
        hash_map_ = std::make_unique<HashMap<std::shared_ptr<Cell> > >();
        hash_map_->reserve(pts.size());
    }

    // Project the points to the map frame
    for (size_t i = 0; i < pts.size(); i++)
    {
        // Check if point is a number
        bool is_a_number = std::isfinite(pts[i].x) && std::isfinite(pts[i].y) && std::isfinite(pts[i].z);
        if(!is_a_number)
        {
            continue;
        }
        float range = pts[i].vec3f().norm();
        if( (range < opt_.min_range) || (range > opt_.max_range))
        {
            continue;
        }

        Vec3 temp_pt = pose.block<3,3>(0,0)*pts[i].vec3() + pose.block<3,1>(0,3);

        GridIndex index = getGridIndex(temp_pt);
        updateBounds(temp_pt);

        if (hash_map_->count(index) == 0)
        {
            std::shared_ptr<Cell> cell_ptr = std::make_shared<Cell>(index, temp_pt, hyperparameters_, scan_counter_, pts[i].t, pose.block<3,1>(0,3), this);
            hash_map_->insert({index, cell_ptr});
            num_cells_++;
            temp_pt = getCenterPt(index);
            phtree_.emplace({temp_pt[0], temp_pt[1], temp_pt[2]}, cell_ptr);
            if(has_color_)
            {
                cell_ptr->addColor(pts[i].r, pts[i].g, pts[i].b);
            }
        }
        else
        {
            hash_map_->at(index)->addPt(temp_pt, pose.block<3,1>(0,3));
            if(has_color_)
            {
                hash_map_->at(index)->addColor(pts[i].r, pts[i].g, pts[i].b);
            }
        }
    }
    sw.stop();
    sw.print("Time to transform and add points");
}



double MapDistField::getAvgTime(const Vec3& pt)
{
    auto nn = phtree_.begin_knn_query(1, {pt[0], pt[1], pt[2]}, DistancePh()); 
    return nn.second()->getAvgTime();
}
    




std::vector<Pointf> MapDistField::getPts()
{
    std::vector<Pointf> pts;
    if(!hash_map_)
    {
        return pts;
    }
    pts.reserve(num_cells_);
    for (auto& pair : *hash_map_)
    {
        Vec3 pt = pair.second->getPt();
        int count = pair.second->getCount();
        pts.push_back(Pointf(float(pt[0]), float(pt[1]), float(pt[2]), 0.0, count));
        if(has_color_)
        {
            auto [r, g, b] = pair.second->getColor();
            pts.back().r = r;
            pts.back().g = g;
            pts.back().b = b;
            pts.back().has_color = true;
        }
    }
    return pts;
}

std::pair<std::vector<Pointf>, std::vector<Vec3> > MapDistField::getPtsAndNormals(bool clean_behind, bool weighted)
{
    std::vector<Pointf> pts;
    std::vector<Vec3> normals;
    if(!hash_map_)
    {
        return {pts, normals};
    }

    // Get cell in vector form for parallel processing
    std::vector<CellPtr> cells;
    cells.reserve(num_cells_);
    for(auto& pair : *hash_map_)
    {
        cells.push_back(pair.second);
    }


    pts.resize(num_cells_);
    normals.resize(num_cells_);
    #pragma omp parallel for num_threads(16)
    for(size_t i = 0; i < cells.size(); i++)
    {
        Vec3 pt = cells[i]->getPt();
        pts[i] = Pointf(float(pt[0]), float(pt[1]), float(pt[2]), 0.0, cells[i]->getCount(), cells[i]->hasNeighbors());
        if(has_color_)
        {
            auto [r, g, b] = cells[i]->getColor();
            pts[i].r = r;
            pts[i].g = g;
            pts[i].b = b;
            pts[i].has_color = true;
        }
        normals[i] = cells[i]->getNormals({pt}, true, clean_behind, weighted)[0];
    }
    return {pts, normals};
}

std::pair<std::vector<Vec3>, std::vector<Vec3> > MapDistField::getClosestPtAndNormal(const std::vector<Vec3>& pts, const bool clean_behind)
{
    std::vector<Vec3> closest_pts(pts.size());
    std::vector<Vec3> normals(pts.size());
    #pragma omp parallel for num_threads(10)
    for(size_t i = 0; i < pts.size(); i++)
    {
        auto cell = getClosestCell(pts[i]);
        closest_pts[i] = cell->getPt();
        normals[i] = cell->getNormals({pts[i]}, true, clean_behind, false)[0];
        // Check if there is a nan in the normal
        if(!std::isfinite(normals[i][0]) || !std::isfinite(normals[i][1]) || !std::isfinite(normals[i][2]))
        {
            normals[i] = Vec3::Zero();
        }
    }
    return {closest_pts, normals};
}



double MapDistField::queryDistField(const Vec3& pt, const bool field)
{
    double dist = std::numeric_limits<double>::max();
    for(auto nn= phtree_.begin_knn_query(num_neighbors_, {pt[0], pt[1], pt[2]}, DistancePh()); nn != phtree_.end(); ++nn)
    {
        if(field)
        {
            double temp_dist = nn.second()->getDist(pt);
            dist = std::min(dist, temp_dist);
        }
        else
        {
            double temp_dist = (pt - nn.second()->getPt()).norm();
            dist = std::min(dist, temp_dist);
        }
    }    
    return dist;
}

double MapDistField::querySdf(const Vec3& pt)
{
    double dist = std::numeric_limits<double>::max();
    for(auto nn= phtree_.begin_knn_query(num_neighbors_, {pt[0], pt[1], pt[2]}, DistancePh()); nn != phtree_.end(); ++nn)
    {
        double temp_dist = nn.second()->getSdf(pt);
        if(std::abs(temp_dist) < std::abs(dist))
        {
            dist = temp_dist;
        }
    }    
    return dist;
}


std::vector<double> MapDistField::queryDistField(const std::vector<Vec3>& pts, const bool field)
{
    std::vector<double> dists(pts.size());
    #pragma omp parallel for num_threads(12)
    for(size_t i = 0; i < pts.size(); i++)
    {
        dists[i] = queryDistField(pts[i], field);
    }
    return dists;
}

std::vector<double> MapDistField::querySdf(const std::vector<Vec3>& pts)
{
    std::vector<double> dists(pts.size());
    #pragma omp parallel for num_threads(12)
    for(size_t i = 0; i < pts.size(); i++)
    {
        dists[i] = querySdf(pts[i]);
    }
    return dists;
}


std::pair<double, Vec3> MapDistField::queryDistFieldAndGrad(const Vec3& pt, const bool field)
{
    double dist = std::numeric_limits<double>::max();
    Vec3 grad = Vec3::Zero();
    CellPtr best_cell;
    for(auto nn= phtree_.begin_knn_query(num_neighbors_, {pt[0], pt[1], pt[2]}, DistancePh()); nn != phtree_.end(); ++nn)
    {
        double temp_dist;
        if(field)
        {
            temp_dist = nn.second()->getDist(pt);
        }
        else
        {
            temp_dist = (pt - nn.second()->getPt()).norm();
        }
        if(temp_dist < dist)
        {
            dist = temp_dist;
            best_cell = nn.second();
        }
    }
    if(best_cell)
    {
        if(field)
        {
            std::tie(dist, grad) = best_cell->getDistAndGrad(pt);
        }
        else
        {
            grad = (pt - best_cell->getPt()).normalized();
        }
    }
    return {dist, grad};
}





double MapDistField::distToClosestCell(const Vec3& pt) const
{
    double dist = std::numeric_limits<double>::max();
    for(auto nn= phtree_.begin_knn_query(1, {pt[0], pt[1], pt[2]}, DistancePh()); nn != phtree_.end(); ++nn)
    {
        dist = (pt - nn.second()->getPt()).norm();
    }
    return dist;
}


CellPtr MapDistField::getClosestCell(const Vec3& pt) const
{
    CellPtr closest_cell;
    double dist = std::numeric_limits<double>::max();
    for(auto nn= phtree_.begin_knn_query(num_neighbors_, {pt[0], pt[1], pt[2]}, DistancePh()); nn != phtree_.end(); ++nn)
    {
        double temp_dist = (pt - nn.second()->getPt()).norm();
        if(temp_dist < dist)
        {
            dist = temp_dist;
            closest_cell = nn.second();
        }
    }
    return closest_cell;
}




GridIndex MapDistField::getGridIndex(const Vec3& pos)
{
    return std::make_tuple(std::floor(pos[0]*inv_cell_size_), std::floor(pos[1]*inv_cell_size_), std::floor(pos[2]*inv_cell_size_));
}



GridIndex MapDistField::getGridIndex(const Vec2& pos)
{
    return std::make_tuple(std::floor(pos[0]*inv_cell_size_), std::floor(pos[1]*inv_cell_size_), 0);
}

Vec3 MapDistField::getCenterPt(const GridIndex& index)
{
    return Vec3(std::get<0>(index)*cell_size_f_ + half_cell_size_f_, std::get<1>(index)*cell_size_f_ + half_cell_size_f_, std::get<2>(index)*cell_size_f_ + half_cell_size_f_);
}


std::vector<CellPtr> MapDistField::getNeighborCells(const Vec3& pt)
{
    std::vector<CellPtr> neighbors;
    double radius = (opt_.neighborhood_size+0.5)*cell_size_;
    PhNeighborQuery query;
    phtree_.for_each(query, improbable::phtree::FilterSphere({pt[0], pt[1], pt[2]}, radius, phtree_.converter()));
    for(auto& neighbor : query.neighbors)
    {
        neighbors.push_back(neighbor.second);
    }
    return neighbors;
}



void MapDistField::writeMap(const std::string& filename)
{
    std::cout << "Writing map to file: " << filename << std::endl;
    std::cout << "Querying points and normals ...." << std::endl;

    StopWatch sw;
    sw.start();
    std::vector<Pointf> pts;
    std::vector<Vec3> normals;
    if(opt_.output_normals || opt_.output_mesh)
    {
        std::tie(pts, normals) = getPtsAndNormals(true, opt_.poisson_weighted);
    }
    else
    {
        pts = getPts();
    }

    if(opt_.output_normals || opt_.output_mesh)
    {
        std::vector<Eigen::Matrix<Real,3,1> > pts_eigen;
        pts_eigen.reserve(pts.size());
        std::vector<Eigen::Matrix<Real,3,1> > normals_eigen;
        std::vector<std::array<unsigned char, 3> > colors;
        normals_eigen.reserve(normals.size());
        std::array<Real, 3> min_bounds = {std::numeric_limits<Real>::max(), std::numeric_limits<Real>::max(), std::numeric_limits<Real>::max()};
        std::array<Real, 3> max_bounds = {std::numeric_limits<Real>::lowest(), std::numeric_limits<Real>::lowest(), std::numeric_limits<Real>::lowest()};
        for(size_t i = 0; i < pts.size(); i++)
        {
            if(pts[i].channel && std::isfinite(normals[i][0]) && std::isfinite(normals[i][1]) && std::isfinite(normals[i][2]))
            {
                pts_eigen.push_back(pts[i].vec3().cast<Real>());
                normals_eigen.push_back(normals[i].normalized().cast<Real>());

                if(has_color_)
                {
                    colors.push_back({pts[i].r, pts[i].g, pts[i].b});
                }

                for(int j = 0; j < 3; j++)
                {
                    min_bounds[j] = std::min(min_bounds[j], pts_eigen.back()[j]);
                    max_bounds[j] = std::max(max_bounds[j], pts_eigen.back()[j]);
                }
            }
        }
        sw.stop();
        sw.print("Points and normals queried in");

        sw.reset();
        sw.start();
        happly::PLYData ply_out;
        ply_out.addVertexPositions(pts_eigen);
        ply_out.addVertexNormals(normals_eigen);
        if(has_color_)
        {
            ply_out.addVertexColors(colors);
        }
        ply_out.write(filename, happly::DataFormat::Binary);
        sw.stop();
        sw.print("Time to write map with normals to file ");


        if(opt_.output_mesh)
        {
            sw.reset();
            sw.start();

            std::cout << "Reconstructing mesh ...." << std::endl;
            // Get max range
            Real max_range = 0;
            for(size_t i = 0; i < pts.size(); i++)
            {
                max_range = std::max(max_range, pts[i].vec3().norm());
            }
            // Get the octree depth
            int octree_depth = std::ceil(std::log2(max_range/(cell_size_/3.0)));
            std::cout << "Octree depth: " << octree_depth << std::endl;
            PoissonReconWrapped poisson_recon(pts_eigen, normals_eigen, octree_depth, 5, 1.1, opt_.meshing_point_per_node, opt_.poisson_weighted);
            auto [mesh_v, mesh_f] = poisson_recon.reconstruct();
            std::vector<std::array<unsigned char, 3> > mesh_colors(mesh_v.size(), {0, 0, 0});
            sw.stop();
            sw.print("Time to reconstruct mesh");

            sw.reset();
            sw.start();

            // Ugly fix for the mesh color: fetch the color from the closest point of the vertex
            if(has_color_)
            {
                #pragma omp parallel for num_threads(16)
                for(size_t i = 0; i < mesh_v.size(); i++)
                {
                    CellPtr closest_cell;
                    for(auto nn= phtree_.begin_knn_query(1, {mesh_v[i][0], mesh_v[i][1], mesh_v[i][2]}, DistancePh()); nn != phtree_.end(); ++nn)
                    {
                        closest_cell = nn.second();
                    }
                    if(closest_cell)
                    {
                        auto [r, g, b] = closest_cell->getColor();
                        mesh_colors[i] = {r, g, b};
                    }
                    else
                    {
                        mesh_colors[i] = {0,0,0};
                    }
                }
            }
                    
                


            if(opt_.clean_mesh_threshold > 0.0)
            {
                // Cleaning the mesh
                int num_threads = 16;
                std::vector<std::unordered_set<int>> face_to_remove_per_threads(num_threads);
                std::vector<std::pair<size_t, size_t>> threads_ids;
                size_t num_faces_per_thread = mesh_f.size()/num_threads;
                for(int i = 0; i < num_threads; i++)
                {
                    threads_ids.push_back({i*num_faces_per_thread, (i+1)*num_faces_per_thread});
                }

                #pragma omp parallel for num_threads(num_threads)
                for(int t = 0; t < num_threads; ++t)
                {
                    for(size_t i = threads_ids[t].first; i < threads_ids[t].second; i++)
                    {
                        bool reject = false;
                        Eigen::Matrix<Real,3,1> centroid = Eigen::Matrix<Real,3,1>::Zero();
                        for(int j = 0; j < 3; j++)
                        {
                            auto pt = mesh_v[mesh_f[i][j]];
                            centroid += pt;
                            if(distToClosestCell(Vec3(pt[0],pt[1],pt[2])) > 2.0*cell_size_)
                            {
                                reject = true;
                                break;
                            }
                        }
                        if(!reject)
                        {
                            centroid /= 3.0;
                            if(distToClosestCell(Vec3(centroid[0], centroid[1], centroid[2])) > 2.0*cell_size_)
                            {
                                reject = true;
                            }
                        }
                        if(reject)
                        {
                            face_to_remove_per_threads[t].insert(i);
                        }
                    }
                }

                std::unordered_set<int> face_to_remove;
                for(int i = 0; i < num_threads; i++)
                {
                    for(auto& face : face_to_remove_per_threads[i])
                    {
                        face_to_remove.insert(face);
                    }
                }




                ankerl::unordered_dense::map<int, int> v_id_map;
                std::vector<Eigen::Matrix<Real,3,1> > mesh_v_clean;
                std::vector<Eigen::Matrix<int,3,1> > mesh_f_clean;
                std::vector<std::array<unsigned char, 3> > mesh_colors_clean;
                int v_id = 0;
                for(size_t i = 0; i < mesh_f.size(); i++)
                {
                    if(face_to_remove.count(i) == 0)
                    {
                        Eigen::Matrix<int,3,1> face;
                        for(int j = 0; j < 3; j++)
                        {
                            if(v_id_map.count(mesh_f[i][j]) == 0)
                            {
                                v_id_map[mesh_f[i][j]] = v_id;
                                mesh_v_clean.push_back(mesh_v[mesh_f[i][j]]);
                                mesh_colors_clean.push_back(mesh_colors[mesh_f[i][j]]);
                                face[j] = v_id;
                                v_id++;
                            }
                            else
                            {
                                face[j] = v_id_map[mesh_f[i][j]];
                            }
                        }
                        mesh_f_clean.push_back(face);
                    }
                }



                sw.stop();
                sw.print("Time to clean mesh");

                mesh_v = mesh_v_clean;
                mesh_f = mesh_f_clean;
                mesh_colors = mesh_colors_clean;
            }


            sw.reset();
            sw.start();
            happly::PLYData ply_out_mesh;
            //ply_out_mesh.addVertexPositions(mesh_v);
            //ply_out_mesh.addFaceIndices(mesh_f);
            ply_out_mesh.addVertexPositions(mesh_v);
            ply_out_mesh.addFaceIndices(mesh_f);
            if(has_color_)
            {
                ply_out_mesh.addVertexColors(mesh_colors);
            }
            std::string folder_name = filename.substr(0, filename.find_last_of("/"));
            ply_out_mesh.write(folder_name + "/mesh.ply", happly::DataFormat::Binary);

            sw.stop();
            sw.print("Time to write mesh to file");
        }
    }
    else
    {
        std::vector<Eigen::Matrix<Real,3,1> > pts_eigen;
        std::vector<std::array<unsigned char, 3> > colors;
        pts_eigen.reserve(pts.size());
        for(size_t i = 0; i < pts.size(); i++)
        {
            Vec3f pt = pts[i].vec3();
            pts_eigen.push_back(Eigen::Matrix<Real,3,1>(pt[0], pt[1], pt[2]));
            if(has_color_)
            {

                colors.push_back({pts[i].r, pts[i].g, pts[i].b});
            }
        }

        sw.reset();
        sw.start();
        happly::PLYData ply_out;
        ply_out.addVertexPositions(pts_eigen);
        if(has_color_)
        {
            ply_out.addVertexColors(colors);
        }
        ply_out.write(filename, happly::DataFormat::Binary);
        sw.stop();
        sw.print("Time to write map to file ");

    }





}





RegistrationCostFunction::RegistrationCostFunction(const std::vector<Vec3>& pts, const Mat4& prior, MapDistField* map, const std::vector<double>& weights, const double cauchy_loss_scale, const bool use_field, const bool use_loss)
    : pts_(pts)
    , prior_(prior)
    , map_(map)
    , weights_(weights)
    , use_field_(use_field)
{
    if(use_loss)
    {
        loss_function_ = std::make_unique<ceres::CauchyLoss>(cauchy_loss_scale);
    }
    else
    {
        loss_function_ = std::make_unique<ceres::TrivialLoss>();
    }
    set_num_residuals(pts.size());
    std::vector<int>* parameter_block_sizes = mutable_parameter_block_sizes();
    parameter_block_sizes->push_back(6);
}

void RegistrationCostFunction::setUseField(const bool use_field)
{
    use_field_ = use_field;
}

bool RegistrationCostFunction::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
{
    Eigen::Map<const Vec3> pos(parameters[0]);
    Eigen::Map<const Vec3> rot(parameters[0]+3);

    Mat3 R_prior = prior_.block<3,3>(0,0);
    Vec3 pos_prior = prior_.block<3,1>(0,3);

    Mat3 R = expMap(rot);

    Mat3 R_w = R_prior*R;
    Vec3 p_w = R_prior*pos + pos_prior;

    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> pts(pts_[0].data(), 3, pts_.size());

    if(jacobians != NULL)
    {
        if( jacobians[0] != NULL)
        {
            MatX pts_corr = R*pts;
            pts_corr.colwise() += pos;
            
            MatX pts_w = R_prior*pts_corr;
            pts_w.colwise() += pos_prior;

            Mat3 J_rot = jacobianLefthandSO3(rot);

            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[0], pts_.size(), 6);

            #pragma omp parallel for num_threads(16)
            for(size_t i = 0; i < pts_.size(); i++)
            {
                Vec3 temp_pt = pts_w.col(i);
                auto [dist, grad] = map_->queryDistFieldAndGrad(temp_pt, use_field_);
                // Apply the loss function
                std::array<double, 3> temp;
                loss_function_->Evaluate(dist, temp.data());
                residuals[i] = temp[0] * weights_[i];

                Row3 d_dist_d_rot = -temp[1]*grad.transpose()*R_prior*(toSkewSymMat(pts_corr.col(i)-pos))*J_rot;
                Row3 d_dist_d_pos = temp[1]*grad.transpose()*R_prior;

                jacobian.block<1,3>(i, 0) = weights_[i] * d_dist_d_pos;
                jacobian.block<1,3>(i, 3) = weights_[i] * d_dist_d_rot;
            }
        }
    }
    else
    {
        MatX pts_w = R_w*pts;
        pts_w.colwise() += p_w;

        #pragma omp parallel for num_threads(16)
        for(size_t i = 0; i < pts_.size(); i++)
        {
            residuals[i] = map_->queryDistField(pts_w.col(i), use_field_);
            // Apply the loss function
            std::array<double, 3> temp;
            loss_function_->Evaluate(residuals[i], temp.data());
            residuals[i] = temp[0] * weights_[i];
        }

    }
    return true;
}

