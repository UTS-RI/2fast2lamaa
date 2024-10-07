#pragma once

#include "lice/types.h"
#include "lice/utils.h"
#include "KDTree.h"

#include <unordered_map>
#include <unordered_set>
#include <tuple>

#include "ankerl/unordered_dense.h"


typedef std::tuple<int, int, int> GridIndex;

const float kInvalidDynamicScore = 100.0;
const float kValidDynamicScore = -100.0;

template <typename V>
//using GridHashMap = std::unordered_map<GridIndex, V, boost::hash<GridIndex> >;
using GridHashMap = ankerl::unordered_dense::map<GridIndex, V>;



class FastCell
{
    public:
        bool neighbor_dynamic = false;
        FastCell(const Pointf& pt, const GridIndex key, const bool is_mid, const float lidar_period = 0.1) 
        {
            key_ = key;
            count_ = 0;
            pts_sum_ = Vec4f::Zero();
            cov_sum_ = Mat4f::Zero();
            t_ref_ = pt.t - lidar_period/2.0;
            lidar_period_ = lidar_period;
            addPoint(pt, is_mid);
        }

        void addPoint(const Pointf& pt, const bool is_mid)
        {
            Vec4f temp_pt;
            temp_pt[0] = pt.x;
            temp_pt[1] = pt.y;
            temp_pt[2] = pt.z;
            temp_pt[3] = int(pt.t/lidar_period_) * lidar_period_;

            pts_sum_ += temp_pt;

            cov_sum_ += (temp_pt * temp_pt.transpose());

            min_time_ = std::min(min_time_, temp_pt[3]);
            max_time_ = std::max(max_time_, temp_pt[3]);

            is_mid_ = (is_mid || is_mid_);

            count_++;
        }

        Pointf getCentroid() const
        {
            Pointf centroid;
            centroid.x = pts_sum_[0] / count_;
            centroid.y = pts_sum_[1] / count_;
            centroid.z = pts_sum_[2] / count_;
            return centroid;
        }

        void setDynamic(const uint8_t dynamic, const float dynamic_score)
        {
            dynamic_ = dynamic;
            dynamic_score_ = dynamic_score;
        }

        Vec4f getCentroidSum() const
        {
            return pts_sum_;
        }

        Mat4f getCovSum() const
        {
            return cov_sum_;
        }

        int getCount() const
        {
            return count_;
        }

        GridIndex getKey() const
        {
            return key_;
        }

        float getDynamicScore() const
        {
            if(neighbor_dynamic)
            {
                return kInvalidDynamicScore;
            }
            return dynamic_score_;
        }

        uint8_t getDynamic() const
        {
            if (neighbor_dynamic)
            {
                return 1;
            }
            return dynamic_;
        }

        std::pair<float, float> getTimeRange() const
        {
            return std::make_pair(min_time_, max_time_);
        }

        bool isMid() const
        {
            return is_mid_;
        }



    private:
        Vec4f pts_sum_;
        Mat4f cov_sum_;
        float min_time_ = std::numeric_limits<float>::max();
        float max_time_ = std::numeric_limits<float>::min();
        int count_;
        uint8_t dynamic_;
        float dynamic_score_;
        GridIndex key_;
        bool is_mid_ = false;
        float t_ref_;
        float lidar_period_;

};
typedef std::shared_ptr<FastCell> FastCellPtr;




class FastDynamicMap
{
    public:
        FastDynamicMap(const std::vector<Pointf>& pts, const float voxel_size, const float dynamic_thr, const int mid_id)
            : voxel_size_(voxel_size)
            , half_voxel_size_(voxel_size / 2.0)
            , dynamic_thr_(dynamic_thr)
            , mid_id_(mid_id)
            , pts_(pts)
        {
            mid_pts_.reserve(pts.size());
            //in_mid_.reserve(pts.size());
            cells_.reserve(pts.size()/3);
            //associated_cell_.resize(pts.size());

            for(size_t i = 0; i < pts.size(); ++i)
            {
                GridIndex key = getGridIndex(pts[i]);
                bool is_mid = pts[i].scan_id == mid_id_;
                
                if(cells_.find(key) == cells_.end())
                {
                    auto new_cell = std::make_shared<FastCell>(pts[i], key, is_mid);
                    cells_[key] = new_cell;
                    if (is_mid)
                    {
                        mid_pts_.push_back(std::make_pair(i, new_cell));
                    }
                }
                else
                {
                    auto& cell = cells_[key];
                    cell->addPoint(pts[i], is_mid);
                    if(is_mid)
                    {
                        mid_pts_.push_back(std::make_pair(i, cell));
                    }
                }
            }
        }

        std::vector<Pointf> getDownSampledPoints() const
        {
            std::vector<Pointf> down_sampled_points;
            for(const auto& cell : cells_)
            {
                Pointf centroid = cell.second->getCentroid();
                centroid.i = cell.second->getDynamicScore();
                centroid.dynamic = cell.second->getDynamic();
                down_sampled_points.push_back(centroid);
            }
            return down_sampled_points;
        }




        std::tuple<std::vector<Pointf>, std::vector<Pointf>, std::vector<Pointf>> getDynamicPoints() const
        {
            std::vector<Pointf> dynamic_pts;
            dynamic_pts.reserve(mid_pts_.size());
            std::vector<Pointf> static_pts;
            static_pts.reserve(mid_pts_.size());
            std::vector<Pointf> unsure_pts;
            unsure_pts.reserve(mid_pts_.size());
            for(const auto& mid_pt : mid_pts_)
            {
                Pointf pt = pts_[mid_pt.first];
                pt.dynamic = mid_pt.second->getDynamic();
                bool neighbor_dynamic = mid_pt.second->neighbor_dynamic;
                if(pt.dynamic == 1 || neighbor_dynamic)
                {
                    dynamic_pts.push_back(std::move(pt));
                }
                else if(pt.dynamic == 0)
                {
                    static_pts.push_back(std::move(pt));
                    //static_pts.back().i = mid_pt.second->getDynamicScore();
                }
                else
                {
                    unsure_pts.push_back(std::move(pt));
                }
            }
            return std::make_tuple(dynamic_pts, static_pts, unsure_pts);



            //std::vector<Pointf> dynamic_pts;
            //std::vector<Pointf> static_pts;
            //std::vector<Pointf> unsure_pts;

            //for(size_t i = 0; i < pts_.size(); ++i)
            //{
            //    if(in_mid_[i])
            //    {
            //        Pointf pt = pts_[i];
            //        const GridIndex key = getGridIndex(pt);
            //        FastCell* cell = cells_.at(key).get();
            //        pt.dynamic = cell->getDynamic();

            //        if(pt.dynamic == 1)
            //        {
            //            dynamic_pts.push_back(std::move(pt));
            //        }
            //        else if(pt.dynamic == 0)
            //        {
            //            static_pts.push_back(std::move(pt));
            //        }
            //        else
            //        {
            //            unsure_pts.push_back(std::move(pt));
            //        }
            //    }
            //}
            //return std::make_tuple(dynamic_pts, static_pts, unsure_pts);
        }



        void computeDynamicScore() const
        {
            // Compute number of voxels in search radius
            //int nb_voxels = int(std::ceil(search_radius / voxel_size_));
            //float search_radius_sq = search_radius * search_radius;
            const float kMinTimeDiff = 0.03;
            const float kMaxThickness = 0.02*0.02;
            const float kMinSpread = (voxel_size_/3.0)*(voxel_size_/3.0);
            const float kMinCos = std::cos(88.0*M_PI/180.0);

            std::vector<FastCell*> cells_vector;
            cells_vector.reserve(cells_.size());
            for(const auto& cell : cells_)
            {
                if(cell.second->isMid())
                {
                    cells_vector.push_back(cell.second.get());
                }
            }

            for(size_t i = 0; i < cells_vector.size(); ++i)
            {
                std::vector<FastCell*> neighbors;

                // Get the neighbors
                GridIndex current_key = cells_vector[i]->getKey();
                for(int x = -1; x <= 1; ++x)
                {
                    for(int y = -1; y <= 1; ++y)
                    {
                        for(int z = -1; z <= 1; ++z)
                        {
                            if( x == 0 && y == 0 && z == 0)
                            {
                                continue;
                            }
                            GridIndex key = {std::get<0>(current_key) + x, std::get<1>(current_key) + y, std::get<2>(current_key) + z};
                            if(cells_.find(key) != cells_.end())
                            {
                                neighbors.push_back(cells_.at(key).get());
                            }
                        }
                    }
                }
                auto [min_time, max_time] = cells_vector[i]->getTimeRange();
                Vec4f center_sum = cells_vector[i]->getCentroidSum();
                Mat4f cov_sum = cells_vector[i]->getCovSum();
                int count = cells_vector[i]->getCount();
                for(const auto& cell : neighbors)
                {
                    std::pair<float, float> time_range = cell->getTimeRange();
                    min_time = std::min(min_time, time_range.first);
                    max_time = std::max(max_time, time_range.second);

                    center_sum += cell->getCentroidSum();
                    cov_sum += cell->getCovSum();
                    count += cell->getCount();
                }

                uint8_t dynamic = 2;
                float dynamic_score = kInvalidDynamicScore;
                if((max_time - min_time) > kMinTimeDiff)
                {

                    Vec4f center_mean = center_sum / count;
                    Mat4f cov_mean = cov_sum / count - center_mean * center_mean.transpose();
                    Mat3f cov_space = cov_mean.block<3, 3>(0, 0);

                    // Transform the covariance matrix to a correlation matrix
                    Vec4f diag = cov_mean.diagonal().array().sqrt();
                    float max_spatial_std = std::sqrt(cov_mean.diagonal().segment<3>(0).sum());
                    diag[0] = max_spatial_std;
                    diag[1] = max_spatial_std;
                    diag[2] = max_spatial_std;
                    //diag[3] = 1;
                    Vec4f inv_diag = diag.array().inverse().matrix();
                    cov_mean = inv_diag.asDiagonal() * cov_mean * inv_diag.asDiagonal();

                    dynamic_score = cov_mean.block<3,1>(0,3).norm();

                    if(dynamic_score > dynamic_thr_)
                    {
                        // Check that the points are not on a plane
                        Eigen::SelfAdjointEigenSolver<Mat3f> eigen_solver(cov_space);
                        Vec3f eigenvalues = eigen_solver.eigenvalues();
                        // Get smallest eigenvalue and associated eigenvector
                        int min_idx = 0;
                        eigenvalues.minCoeff(&min_idx);
                        Vec3f normal = eigen_solver.eigenvectors().col(min_idx);
                        float min_eigenvalue = eigenvalues[min_idx];
                        float second_min_eigenvalue = std::min(eigenvalues[(min_idx + 1) % 3], eigenvalues[(min_idx + 2) % 3]);
                        Vec3f dir = center_mean.segment<3>(0).normalized();
                        if((min_eigenvalue < kMaxThickness) && (second_min_eigenvalue > kMinSpread) && (std::abs(normal.dot(dir)) > kMinCos))
                        {
                            dynamic = 0;
                            dynamic_score = kValidDynamicScore;
                        }
                        else
                        {
                            for(const auto& neighbor : neighbors)
                            {
                                neighbor->neighbor_dynamic = true;
                            }
                            dynamic = 1;
                        }
                    }
                    else
                    {
                        dynamic = 0;
                    }
                }



                ////////////
                //auto [min_time, max_time] = cells_vector[i]->getTimeRange();
                //uint8_t dynamic = 2;
                //float dynamic_score = kInvalidDynamicScore;
                //GridIndex current_key = cells_vector[i]->getKey();
                //std::vector<FastCell*> neighbors;

                ////auto nn = kdtree_.searchBall({centroid.x, centroid.y, centroid.z}, search_radius_sq);
                //// Get the neighbors
                //for(int x = -1; x <= 1; ++x)
                //{
                //    for(int y = -1; y <= 1; ++y)
                //    {
                //        for(int z = -1; z <= 1; ++z)
                //        {
                //            if( x == 0 && y == 0 && z == 0)
                //            {
                //                continue;
                //            }
                //            GridIndex key = {std::get<0>(current_key) + x, std::get<1>(current_key) + y, std::get<2>(current_key) + z};
                //            if(cells_.find(key) != cells_.end())
                //            {
                //                neighbors.push_back(cells_.at(key).get());
                //            }
                //        }
                //    }
                //}



                //if((neighbors.size() > 1))// && (max_time - min_time) > kMinTimeDiff)
                //{
                //    Vec4f center_sum = cells_vector[i]->getCentroidSum();
                //    Mat4f cov_sum = cells_vector[i]->getCovSum();
                //    int count = cells_vector[i]->getCount();
                //    for(const auto& cell : neighbors)
                //    {
                //        std::pair<float, float> time_range = cell->getTimeRange();
                //        min_time = std::min(min_time, time_range.first);
                //        max_time = std::max(max_time, time_range.second);

                //        center_sum += cell->getCentroidSum();
                //        cov_sum += cell->getCovSum();
                //        count += cell->getCount();
                //    }

                //    if((max_time - min_time) < kMinTimeDiff)
                //    {
                //        dynamic = 2;
                //        dynamic_score = kInvalidDynamicScore;
                //    }
                //    else
                //    {

                //        Vec4f center_mean = center_sum / count;
                //        Mat4f cov_mean = cov_sum / count - center_mean * center_mean.transpose();

                //        Eigen::SelfAdjointEigenSolver<Mat3f> space_eigen_solver(cov_mean.block<3, 3>(0, 0));
                //        Vec3f space_eigenvalues = space_eigen_solver.eigenvalues();
                //        // Get smallest eigenvalue and associated eigenvector
                //        int space_min_eigenvalue_idx = 0;
                //        space_eigenvalues.minCoeff(&space_min_eigenvalue_idx);
                //        Vec3f space_normal = space_eigen_solver.eigenvectors().col(space_min_eigenvalue_idx);

                //        Eigen::SelfAdjointEigenSolver<Mat4f> eigensolver(cov_mean);
                //        Vec4f eigenvalues = eigensolver.eigenvalues();
                //        int min_eigenvalue_idx = 0;
                //        eigenvalues.minCoeff(&min_eigenvalue_idx);
                //        Vec4f min_eigenvector = eigensolver.eigenvectors().col(min_eigenvalue_idx);


                //        dynamic_score = std::abs(space_normal.dot(min_eigenvector.segment<3>(0)));

                //        if(dynamic_score < dynamic_thr_)
                //        {
                //            dynamic = 1;
                //            for(const auto& neighbor : neighbors)
                //            {
                //                neighbor->neighbor_dynamic = true;
                //            }
                //        }
                //        else
                //        {
                //            dynamic = 0;
                //        }


                //    }
                //}
                cells_vector[i]->setDynamic(dynamic, dynamic_score);
            }
        }


    private:
        float voxel_size_;
        float half_voxel_size_;
        float dynamic_thr_;
        int mid_id_;
        //std::vector<bool> in_mid_;
        std::vector<std::pair<int,FastCellPtr>> mid_pts_;
        const std::vector<Pointf>& pts_;
        //jk::tree::KDTree<FastCellPtr, 3, 16, jk::tree::SquaredL2, float> kdtree_;


        GridHashMap<FastCellPtr> cells_;

        GridIndex getGridIndex(const Pointf& pt) const
        {
            return std::make_tuple(int(pt.x / voxel_size_), int(pt.y / voxel_size_), int(pt.z / voxel_size_));
        }

        std::array<float, 3> getGridCenter(const GridIndex& key) const
        {
            return {std::get<0>(key) * voxel_size_ + half_voxel_size_, std::get<1>(key) * voxel_size_ + half_voxel_size_, std::get<2>(key) * voxel_size_ + half_voxel_size_};
        }
};





inline std::tuple<std::vector<Pointf>, std::vector<Pointf>, std::vector<Pointf>> dynamicFiltering(const std::vector<Pointf>& pts, const float dynamic_thr, const float voxel_size)
{
    std::vector<Pointf> dynamic_pts;
    std::vector<Pointf> static_pts;
    std::vector<Pointf> unsure_pts;

    int mid_id = (pts[0].scan_id + pts.back().scan_id) / 2;

    std::cout << "Mid id: " << mid_id << std::endl;
    StopWatch sw2;
    sw2.start();


    FastDynamicMap dynamic_map(pts, voxel_size, dynamic_thr, mid_id);

    sw2.stop();
    sw2.print("Dynamic map creation time");

    sw2.reset();
    sw2.start();

    dynamic_map.computeDynamicScore();
    sw2.stop();
    sw2.print("Dynamic score computation time");

    sw2.reset();
    sw2.start();
    std::tie(dynamic_pts, static_pts, unsure_pts) = dynamic_map.getDynamicPoints();
    sw2.stop();
    sw2.print("Get dynamic points time");

    return std::make_tuple(static_pts, dynamic_pts, unsure_pts);
}