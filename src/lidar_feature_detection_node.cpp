#include <rclcpp/rclcpp.hpp>
#include "lice/ros_utils.h"
#include "lice/types.h"
#include "lice/utils.h"
#include "lice/math_utils.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "lice/lidar_front_end.h"

#include "ankerl/unordered_dense.h"

class LidarFeatureNode : public rclcpp::Node
{
    public:
        LidarFeatureNode()
            : Node("lidar_feature_node")
        {
            RCLCPP_INFO(this->get_logger(), "Starting lidar_feature_node node");
            max_dist_ = float(readRequiredFieldDouble(this, "max_dist"));
            min_dist_ = float(readRequiredFieldDouble(this, "min_dist"));
            max_planar_pts_ = readFieldInt(this, "max_planar_pts", 1000);
            std::string channel_elevation_str = readFieldString(this, "channel_elevation", "");

            pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_features", 10);
            sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar_scan", 10, std::bind(&LidarFeatureNode::pcCallback, this, std::placeholders::_1));

            if(channel_elevation_str != "")
            {
                channel_elevation_ = splitString(channel_elevation_str, ",");
                for(size_t i = 0; i < channel_elevation_.size(); ++i)
                    channel_elevation_[i] *= M_PI/180.0;
            }
        }

    private:
        float max_dist_;
        float min_dist_;
        int max_planar_pts_;
        std::vector<float> channel_elevation_;
        bool has_intensity_;
        bool has_channel_;
        size_t win_size_ = 6;
        size_t win_size_quarter_ = std::max(win_size_/4, (size_t)1);
        size_t win_size_half_ = std::max(win_size_/2, win_size_quarter_+1);
        float planar_thr_ = 0.03;
        bool first_ = true;
        float median_dt_;



        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;



        void pcCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            std::vector<Pointf> pts = pointCloud2MsgToPtsVecInternalFloat(msg);

            has_channel_ = pts[0].channel != kNoChannel;
            // Split the point cloud into channels and remove points too close
            std::vector<std::vector<Pointf> > channels;
            if(!has_channel_ && (channel_elevation_.size() == 0))
            {
                if(min_dist_ > 0.0)
                {
                    channels.push_back(std::vector<Pointf>());
                    channels[0].reserve(pts.size());
                    for(size_t i = 0; i < pts.size(); ++i)
                    {
                        if( (pts[i].vec3().norm() > min_dist_) && (pts[i].vec3().norm() < max_dist_) )
                            channels[0].push_back(pts[i]);
                    }
                }
                else
                {
                    channels.push_back(pts);
                }
            }
            else if(!has_channel_)
            {
                channels = splitPointCloud(pts, channel_elevation_, min_dist_, max_dist_);
            }
            else
            {
                channels = splitPointCloud(pts, min_dist_, max_dist_);
            }

            if(first_)
            {
                // Get the median Dt for all channels
                std::vector<float> dt;
                for(size_t i = 0; i < channels.size(); ++i)
                {
                    dt.push_back(getMedianDt(channels[i]));
                }
                std::sort(dt.begin(), dt.end());
                median_dt_ = dt[dt.size()/2];
                if(median_dt_ <= 0.0)
                {
                    dt.clear();
                    for(size_t i = 0; i < channels.size(); ++i)
                    {
                        dt.push_back(getMeanDt(channels[i]));
                    }
                    std::sort(dt.begin(), dt.end());
                    median_dt_ = dt[dt.size()/2];

                }
                if(median_dt_ <= 0.0)
                {
                    RCLCPP_ERROR(this->get_logger(), "Median dt is zero or negative, cannot compute features");
                    return;
                }
                first_ = false;
            }

            std::vector<Pointf> planar_pts;
            std::vector<Pointf> edge_pts;
            planar_pts.reserve(max_planar_pts_);
            edge_pts.reserve(max_planar_pts_);

            for(size_t i = 0; i < channels.size(); ++i)
            {

                // Get geometric features
                auto [plane_candidates, rough_score] = getPlanarCandidatesAndRoughness(channels[i], median_dt_, win_size_, planar_thr_);
                //std::vector< bool > edge_candidates = getEdgeCandidates(channels[i], median_dt);
                auto [edge_candidates, jump_candidates] = getEdgeCandidates(channels[i], rough_score, win_size_, min_dist_, median_dt_);
                std::vector<Pointf> local_planar_pts;
                for(size_t j = 0; j < plane_candidates.size(); ++j)
                {
                    if(plane_candidates[j])
                    {
                        local_planar_pts.push_back(channels[i][j]);
                        local_planar_pts.back().type = 1;
                    }
                    if(edge_candidates[j])
                    {
                        edge_pts.push_back(channels[i][j]);
                        edge_pts.back().type = 2;
                    }
                }
                local_planar_pts = downsampleChannelRandom(local_planar_pts, max_planar_pts_/channels.size());

                planar_pts.insert(planar_pts.end(), local_planar_pts.begin(), local_planar_pts.end());

                if(channels[i].size() < win_size_ + 1)
                    continue;
                for(size_t j = win_size_half_; j < channels[i].size() - win_size_half_-1; ++j)
                {
                    if(rough_score[j] < planar_thr_)
                        continue;

                    bool valid = true;
                    for (size_t k = j - win_size_half_; k < j + win_size_half_; ++k)
                    {
                        if (plane_candidates[k] || edge_candidates[k])
                        {
                            valid = false;
                            break;
                        }
                    }
                    if(!valid)
                        continue;
                    // Check if the point is a local maximum over a window
                    bool local_maxima = true;
                    for(size_t k = j - win_size_quarter_; k < j + win_size_quarter_+1; ++k)
                    {
                        if(j!=k && rough_score[k] > rough_score[j])
                            local_maxima = false;
                    }

                    if(local_maxima)
                    {
                        if((rough_score[j-win_size_half_] >= 0.0) && (rough_score[j-win_size_half_] < planar_thr_) && (rough_score[j+win_size_half_] >= 0.0) && (rough_score[j+win_size_half_] < planar_thr_))
                        {
                            edge_pts.push_back(channels[i][j]);
                            edge_pts.back().type = 2;
                        }
                    }
                }

            }


            // Downsample the edge points
            const float kVoxelSize = 0.06;
            const float kHalfVoxelSize = kVoxelSize/2.0;
            //std::unordered_map<std::tuple<int, int, int>, std::pair<Pointf,float>, boost::hash<std::tuple<int, int, int>>> edge_pts_map;
            ankerl::unordered_dense::map<std::tuple<int, int, int>, std::pair<Pointf,float> > edge_pts_map;
            for(const auto& pt: edge_pts)
            {
                int x = (int)(pt.x/kVoxelSize);
                int y = (int)(pt.y/kVoxelSize);
                int z = (int)(pt.z/kVoxelSize);
                std::tuple<int, int, int> key = std::make_tuple(x, y, z);
                // Get the voxel center
                Vec3f voxel_center(x*kVoxelSize + kHalfVoxelSize, y*kVoxelSize + kHalfVoxelSize, z*kVoxelSize + kHalfVoxelSize);
                float dist = (pt.vec3() - voxel_center).norm();
                if(edge_pts_map.find(key) == edge_pts_map.end())
                {
                    edge_pts_map[key] = std::make_pair(pt, dist);
                }
                else
                {
                    if(dist < edge_pts_map[key].second)
                    {
                        edge_pts_map[key] = std::make_pair(pt, dist);
                    }
                }
            }
            edge_pts.clear();
            edge_pts.reserve(edge_pts_map.size());
            for(const auto& it: edge_pts_map)
            {
                edge_pts.push_back(it.second.first);
            }




            // Create a new point containing all the features
            int nb_features = planar_pts.size() + edge_pts.size();
            std::vector<Pointf> all_features;
            all_features.reserve(nb_features);
            all_features.insert(all_features.end(), planar_pts.begin(), planar_pts.end());
            all_features.insert(all_features.end(), edge_pts.begin(), edge_pts.end());

            pub_->publish(ptsVecToPointCloud2MsgInternal(all_features, msg->header));
        }

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarFeatureNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


