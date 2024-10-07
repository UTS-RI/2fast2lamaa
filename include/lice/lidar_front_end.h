#pragma once


#include "types.h"
#include <vector>
#include <cmath>
#include <random>


const float kChannelElTolerance = 0.5;

// Function to split a multi-channel point cloud into multiple point clouds (one per channel)
inline std::vector<std::vector<Pointf> > splitPointCloud(const std::vector<Pointf>& pc, const std::vector<float>& channel_elevations, float min_dist = 0.0, float max_dist = 1000.0)
{
    std::vector<std::vector<Pointf> > output(channel_elevations.size());
    for(size_t i = 0; i < pc.size(); ++i)
    {
        if(min_dist > 0.0)
        {
            if((pc[i].vec3().norm() < min_dist) || (pc[i].vec3().norm() > max_dist))
                continue;
        }
        float min_diff = 1000;
        int min_index = -1;
        float el = std::atan2(pc[i].z, std::sqrt(pc[i].x*pc[i].x + pc[i].y*pc[i].y));
        for(size_t j = 0; j < channel_elevations.size(); ++j)
        {
            float diff = std::abs(el - channel_elevations[j]);
            if(diff < min_diff)
            {
                min_diff = diff;
                min_index = j;
            }
        }
        if(min_diff < kChannelElTolerance)
        {
            output[min_index].push_back(pc[i]);
            output[min_index].back().channel = min_index;
        }
    }

    return output;
}

inline std::vector<std::vector<Pointf> > splitPointCloud(const std::vector<Pointf>& pc, float min_dist = 0.0, float max_dist = 1000.0)
{
    std::map<int,int> channel_ids;
    int counter = 0;
    for(size_t i = 0; i < pc.size(); ++i)
    {
        // Check if channel is already in the list
        if (channel_ids.find(pc[i].channel) != channel_ids.end())
            continue;

        if(min_dist > 0.0)
        {
            if((pc[i].vec3().norm() < min_dist) || (pc[i].vec3().norm() > max_dist))
                continue;
        }
        channel_ids[pc[i].channel] = counter;
        counter++;
    }

    std::vector<std::vector<Pointf> > output(counter);
    for(size_t i = 0; i < pc.size(); ++i)
    {
        if(min_dist > 0.0)
        {
            if((pc[i].vec3().norm() < min_dist) || (pc[i].vec3().norm() > max_dist))
                continue;
        }
        output[channel_ids[pc[i].channel]].push_back(pc[i]);
    }
    return output;
}

// Get median time between points in a point cloud channel
inline float getMedianDt(const std::vector<Pointf>& pc)
{
    std::vector<double> dt;
    for(size_t i = 1; i < pc.size(); ++i)
    {
        dt.push_back(pc[i].t - pc[i-1].t);
    }
    std::sort(dt.begin(), dt.end());
    return float(dt[dt.size()/2]);
}


// Get intensity features in a point cloud channel
inline std::tuple<std::vector<Pointf>, std::vector<Pointf>> getIntensityFeatures(const std::vector<Pointf>& pc, const float median_dt, const int win_size = 10)
{
    std::vector<Pointf> features_pos;
    std::vector<Pointf> features_neg;
    int win_size_half = win_size/2;
    float max_dt = median_dt*win_size*1.3;
    //int win_size_quarter = std::max(win_size/4, 1);

    std::vector<float> scores(pc.size());
    std::vector<float> abs_scores(pc.size());
    for(size_t i = win_size_half; i < (pc.size()-win_size_half-1); ++i)
    {
        if ((pc[i+win_size_half].t - pc[i-win_size_half].t) > max_dt)
            continue;

        float i0 = pc[i-win_size_half].i;
        float i1 = pc[i+win_size_half].i;
        scores[i] = std::abs(pc[i].i - ((i0+i1)/2.0));
        abs_scores[i] = scores[i];
    }

    // Get the score threshold as the 75% percentile of the absolute scores
    std::sort(abs_scores.begin(), abs_scores.end());
    float score_thr = 2.0*abs_scores[abs_scores.size()*0.75];

    for(size_t i = win_size_half; i < (pc.size()-win_size_half-1); ++i)
    {
        if(std::abs(scores[i]) < score_thr)
            continue;

        // Check if the point is a local maximum or a local minimum
        bool is_max = true;
        bool is_min = true;
        for(int j = 0; j < win_size_half; ++j)
        {
            if(scores[i+j] > scores[i])
                is_max = false;
            if(scores[i-j] > scores[i])
                is_max = false;

            if(scores[i+j] < scores[i])
                is_min = false;
            if(scores[i-j] < scores[i])
                is_min = false;
        }

        if(is_max)
            features_pos.push_back(pc[i]);
        if(is_min)
            features_neg.push_back(pc[i]);

    }
    return {features_pos, features_neg};
}

// Get index of planar candidates in a point cloud channel
inline std::tuple< std::vector<bool>, std::vector<float> > getPlanarCandidatesAndRoughness(const std::vector<Pointf>& pc, const float median_dt, const size_t win_size = 10, const float planar_thr = 0.03)
{
    if(pc.size() == 0)
        return {std::vector<bool>(), std::vector<float>()};
    if(pc.size() < win_size+1)
        return {std::vector<bool>(pc.size(), false), std::vector<float>(pc.size(), -1.0)};

    float planar_thr_sq = planar_thr*planar_thr;
    float max_dt = median_dt*win_size*1.3;
    size_t win_size_half = win_size/2;
    std::vector<bool> plane_candidates(pc.size(), true);
    std::vector<float> roughness(pc.size(), -1.0);
    for(size_t i = 0; i < win_size_half; ++i)
    {
        plane_candidates.at(i) = false;
    }
    for(size_t i = pc.size()-win_size_half; i < pc.size(); ++i)
    {
        plane_candidates.at(i) = false;
    }
    for(size_t i = win_size_half; i < (pc.size()-win_size_half-1); ++i)
    {
        if ((pc.at(i+win_size_half).t - pc.at(i-win_size_half).t) > max_dt)
            continue;

        // Compute the point to line distance for each point in the window
        Vec3f p1 = pc.at(i-win_size_half).vec3();
        Vec3f p2 = pc.at(i+win_size_half).vec3();
        Vec3f v12 = (p2 - p1).normalized();

        for(size_t j = 0; j < win_size; ++j)
        {
            Vec3f p = pc[i+j-win_size_half].vec3();
            Vec3f v = (p - p1);
            float d_sq= v.cross(v12).squaredNorm();
            if(d_sq > planar_thr_sq)
            {   
                plane_candidates.at(i) = false;
            }
            if(j == win_size_half)
            {
                roughness.at(i) = std::sqrt(d_sq);
            }
        }


    }
    return {plane_candidates, roughness};
}

// Get index of depth jumps in a point cloud channel
inline std::tuple<std::vector<bool>, std::vector<bool> > getEdgeCandidates(const std::vector<Pointf>& pc, const std::vector<float>& roughness, const size_t win_size, const float median_dt, const float min_dist, const float thr = 0.5)
{
    if(pc.size() == 0)
        return {std::vector<bool>(), std::vector<bool>()};
    if(pc.size() < win_size+1)
        return {std::vector<bool>(pc.size(), false), std::vector<bool>(pc.size(), false)};

    int win_size_half = win_size/2;
    float half_thr = thr/2.0;
    std::vector<bool> edge_candidates(pc.size(), false);
    std::vector<bool> jump_candidates(pc.size(), false);
    float t_thr = median_dt*1.3;
    float t_thr_far = 20.0*median_dt*1.3;
    float t_thr2 = 2.0*median_dt*1.3;
    float dist_thr = min_dist + thr;
    for(size_t i = win_size_half; i < pc.size()-win_size_half-1; ++i)
    {
        float dt = pc[i+1].t - pc[i-1].t;
        float dt0 = pc[i].t - pc[i-1].t;
        float dt1 = pc[i+1].t - pc[i].t;

        float r = pc[i].vec3().norm();
        float r_0 = r - pc[i-1].vec3().norm();
        float r_1 = pc[i+1].vec3().norm() - r;
        
        if( (dt1>0.0) && (dt0 > t_thr_far) && (dt1 < t_thr) && (std::abs(r_1) < half_thr) && (roughness[i+win_size_half] < 0.1) && (r > dist_thr) )
        {
            edge_candidates[i] = true;
        }
        else if( (dt0>0.0) && (dt0 < t_thr) && (dt1 > t_thr_far) && (std::abs(r_0) < half_thr) && (roughness[i-win_size_half] < 0.1) && (r > dist_thr) )
        {
            edge_candidates[i] = true;
        }
        else if(dt < t_thr2)
        {
            if(r_0 < -thr && std::abs(r_1) < half_thr)
            {
                if(roughness[i+win_size_half] > 0 && roughness[i+win_size_half] < 0.10)
                {
                    edge_candidates[i] = true;
                }
                else
                {
                    jump_candidates[i] = true;
                }
            }
            else if(std::abs(r_0) < half_thr && r_1 > thr)
            {
                jump_candidates[i] = true;
                if(roughness[i-win_size_half] > 0 && roughness[i-win_size_half] < 0.10)
                {
                    edge_candidates[i] = true;
                }
                else
                {
                    jump_candidates[i] = true;
                }
            }
        }

    }
    return {edge_candidates, jump_candidates};
}
//// Get index of depth jumps in a point cloud channel
//inline std::vector<bool> getEdgeCandidates(const std::vector<Pointd>& pc, const double median_dt, const double thr = 0.5)
//{
//    double half_thr = thr/2.0;
//    std::vector<bool> jump_candidates(pc.size(), false);
//    for(int i = 1; i < pc.size()-1; ++i)
//    {
//        double dt = pc[i+1].t - pc[i-1].t;
//        if(dt < 2*median_dt*1.3)
//        {
//            double r_0 = pc[i].vec3().norm() - pc[i-1].vec3().norm();
//            double r_1 = pc[i+1].vec3().norm() - pc[i].vec3().norm();
//            if(r_0 < -thr && std::abs(r_1) < half_thr)
//            {
//                jump_candidates[i] = true;
//            }
//            else if(std::abs(r_0) < half_thr && r_1 > thr)
//            {
//                jump_candidates[i] = true;
//            }
//        }
//    }
//    return jump_candidates;
//}



// Function to downsample a point cloud channel using random sampling
inline std::vector<Pointf> downsampleChannelRandom(const std::vector<Pointf>& pc, const size_t n)
{
    if(n >= pc.size())
        return pc;

    std::vector<Pointf> pc_ds;
    pc_ds.reserve(n);
    std::vector<int> idxs(pc.size());
    std::iota(idxs.begin(), idxs.end(), 0);

    auto rng = std::default_random_engine {};
    std::shuffle(idxs.begin(), idxs.end(), rng);
    for(size_t i = 0; i < n; ++i)
    {
        pc_ds.push_back(pc[idxs[i]]);
    }
    return pc_ds;
}


