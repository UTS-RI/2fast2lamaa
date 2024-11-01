#include "ros/ros.h"
#include "lice/map_distance_field.h"
#include "lice/ros_utils.h"
#include "lice/utils.h"
#include "lice/math_utils.h"

#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_broadcaster.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "ankerl/unordered_dense.h"

#include "ffastllamaa/QueryDistField.h"


#include <sys/stat.h>


bool folderExists(const std::string& folderPath) {
    struct stat info;
    if (stat(folderPath.c_str(), &info) != 0)
        return false; // Cannot access folder
    else if (info.st_mode & S_IFDIR) // S_IFDIR means it's a directory
        return true; // Folder exists
    else
        return false; // Path exists but it's not a folder
}

bool createFolder(const std::string& folderPath) {
    mode_t mode = 0755; // UNIX style permissions
    int ret = mkdir(folderPath.c_str(), mode);
    if (ret == 0)
        return true; // Folder created successfully
    return false; // Failed to create folder
}



class GpMapNode
{
    public:
        GpMapNode()
        {
            ros::NodeHandle nh("~");

            MapDistFieldOptions options;

            double voxel_size = readRequiredField<double>(nh, "voxel_size");
            options.cell_size = voxel_size;
            downsample_size_ = 3.0 * voxel_size;
            half_voxel_size_sq_ = std::pow(voxel_size / 2.0,2);
            options.neighborhood_size = readRequiredField<int>(nh, "neighbourhood_size");

            register_ = readField<bool>(nh, "register", true);
            bool with_prior = readRequiredField<bool>(nh, "with_prior");
            with_prior_ = with_prior;
            approximate_ = readField<bool>(nh, "register_with_approximate_field", false);

            map_publish_period_ = readField<double>(nh, "map_publish_period", 1.0);

            options.use_temporal_weights = readField<bool>(nh, "use_temporal_weights", false);

            options.free_space_carving_radius = readField<double>(nh, "free_space_carving_radius", -1.0);

            max_nb_pts_ = readField<int>(nh, "max_num_pts_for_registration", 4000);

            options.free_space_carving = false;
            if (options.free_space_carving_radius > 0.0)
            {
                options.free_space_carving = true;
            }
            double min_range = readRequiredField<double>(nh, "min_range");
            options.min_range = min_range;
            options.max_range = readField<double>(nh, "max_range", 1000.0);
            min_range_ = min_range;
            max_range_ = options.max_range;

            key_framing_ = readField<bool>(nh, "key_framing", false);
            key_framing_dist_thr_ = readField<double>(nh, "key_framing_dist_thr", 1.0);
            key_framing_rot_thr_ = readField<double>(nh, "key_framing_rot_thr", 0.1);
            key_framing_time_thr_ = readField<double>(nh, "key_framing_time_thr", 1.0);

            map_path_ = readField<std::string>(nh, "map_path", "");
            map_path_ = map_path_ + (map_path_.back() == '/' ? "" : "/") + "map.ply";
            // If folder does not exist, create it
            std::string folder = map_path_.substr(0, map_path_.find_last_of("/"));
            if(!folderExists(folder))
            {
                if(!createFolder(folder))
                {
                    ROS_ERROR("Could not create folder: %s for map output", folder.c_str());
                    return;
                }
                ROS_INFO("Created folder: %s for map output", folder.c_str());
            }


            options.output_normals = readField<bool>(nh, "output_normals", false);
            options.output_mesh = readField<bool>(nh, "output_mesh", false);
            options.clean_mesh_threshold = readField<double>(nh, "clean_mesh_threshold", 2.0*voxel_size);
            options.meshing_point_per_node = readField<double>(nh, "meshing_point_per_node", 2.0);

            pc_type_internal_ = readField<bool>(nh, "point_cloud_internal_type", false);

            map_ = std::make_unique<MapDistField>(options);
            


            if(with_prior)
            {
                pc_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh, "/points_input", 10);
                pose_sub_ = std::make_unique<message_filters::Subscriber<geometry_msgs::TransformStamped>>(nh, "/pose_input", 10);
                sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::TransformStamped>>(*pc_sub_, *pose_sub_, 20);
                sync_->registerCallback(std::bind(&GpMapNode::pcPriorCallback, this, std::placeholders::_1, std::placeholders::_2));
            }
            else
            {
                sub_ = nh.subscribe("/points_input", 1, &GpMapNode::pcCallback, this);
            }


            map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map", 10);

            odom_map_correction_pub_ = nh.advertise<geometry_msgs::TransformStamped>("/odom_map_correction", 10);

            map_publish_thread_ = std::make_unique<std::thread>(&GpMapNode::mapPublishThread, this);

            query_dist_field_srv_ = nh.advertiseService("/query_dist_field", &GpMapNode::queryDistFieldCallback, this);


        }

        ~GpMapNode()
        {
            running_ = false;
            map_publish_thread_->join();
        }

    private:

        double map_publish_period_ = 1.0;
        bool key_framing_ = false;
        double key_framing_dist_thr_ = 1.0;
        double key_framing_rot_thr_ = 0.1;
        double key_framing_time_thr_ = 1.0;

        double min_range_ = 1.0;
        double max_range_ = 1000.0;

        size_t max_nb_pts_ = 4000;

        std::string map_path_;

        std::unique_ptr<MapDistField> map_;
        std::mutex map_mutex_;

        // Sub for time synchronised prior
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pc_sub_;
        std::unique_ptr<message_filters::Subscriber<geometry_msgs::TransformStamped>> pose_sub_;
        std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::TransformStamped>> sync_;

        // Sub for no prior
        ros::Subscriber sub_;

        // Global map publisher
        ros::Publisher map_pub_;
        ros::Publisher odom_map_correction_pub_;


        ros::ServiceServer query_dist_field_srv_;





        Mat4 current_pose_ = Mat4::Identity();
        
        Mat4 last_input_pose_ = Mat4::Identity();
        Mat4 prior_ = Mat4::Identity();
        bool first_ = true;

        bool register_ = true;

        bool approximate_ = false;
        bool with_prior_ = false;

        double half_voxel_size_sq_;
        double downsample_size_ = 0.4;

        std::atomic<bool> running_ = true;
        std::atomic<int> counter_ = 0;
        int previous_counter_ = 0;

        int last_write_counter_ = 0;

        bool pc_type_internal_ = false;
        ros::Time last_pc_time_;
        double key_framing_time_cumulated_ = 0.0;

        std::unique_ptr<std::thread> map_publish_thread_;

        // Store the last point cloud time
        std::chrono::time_point<std::chrono::high_resolution_clock> last_pc_epoch_time_;
        std::mutex last_pc_epoch_time_mutex_;

        bool queryDistFieldCallback(ffastllamaa::QueryDistField::Request& request, ffastllamaa::QueryDistField::Response& response)
        {
            StopWatch sw;
            sw.start();
            if(request.dim != 3)
            {
                ROS_ERROR("Only 3D points are supported");
                return false;
            }
            std::vector<Vec3> query_pts;
            for(size_t i = 0; i < request.num_pts; i++)
            {
                query_pts.push_back(Vec3(request.pts.at(i*3), request.pts.at(i*3+1), request.pts.at(i*3+2)));
            }
            sw.stop();
            sw.print("QueryDistFieldCallback: Reading request");
            sw.reset();
            sw.start();
            map_mutex_.lock();
            sw.stop();
            sw.print("QueryDistFieldCallback: Locking map");
            sw.reset();
            sw.start();
            std::vector<double> dists = map_->queryDistField(query_pts);
            map_mutex_.unlock();
            sw.stop();
            sw.print("QueryDistFieldCallback: Querying map");
            sw.reset();
            sw.start();
            for(double dist: dists)
            {
                response.dists.push_back(dist);
            }
            sw.stop();
            sw.print("QueryDistFieldCallback: Writing response");

            return true;
        }



        void updateMap(const std::vector<Pointd> pts, const Mat4 trans, ros::Time time)
        {
            if(first_)
            {
                last_input_pose_ = trans;
                prior_ = trans;
                first_ = false;

                map_mutex_.lock();
                map_->addPts(pts, current_pose_);
                map_mutex_.unlock();

                geometry_msgs::TransformStamped temp_msg;
                temp_msg.header.stamp = time;
                temp_msg.header.frame_id = "map";
                temp_msg.child_frame_id = "odom";
                temp_msg.transform = mat4ToTransform(Mat4::Identity());
                odom_map_correction_pub_.publish(temp_msg);
                return;
            }

            StopWatch sw;
            if (register_)
            {
                Mat4 delta_trans = last_input_pose_.inverse() * trans;
                last_input_pose_ = trans;
                prior_ = prior_*delta_trans;

                if(key_framing_)
                {
                    bool compute = false;
                    double time_diff = (ros::Time(time) - ros::Time(last_pc_time_)).toSec();
                    key_framing_time_cumulated_ += time_diff;
                    if(key_framing_time_cumulated_ >= key_framing_time_thr_) compute = true;

                    if(!compute)
                    {
                        auto [dist, rot_diff] = distanceBetweenTransforms(current_pose_, prior_);
                        if( dist >= key_framing_dist_thr_ || rot_diff >= key_framing_rot_thr_) compute = true;
                    }

                    if(!compute) return;
                }                
                sw.start();
                // Downsample the points
                ankerl::unordered_dense::map<GridIndex, std::tuple<Vec3, Vec3, int> > pts_map;
                for(const Pointd& pt: pts)
                {
                    if(!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
                    float range = pt.vec3f().norm();
                    if(range < min_range_) continue;
                    if(range > max_range_) continue;

                    GridIndex idx = {std::floor(pt.x / downsample_size_), std::floor(pt.y / downsample_size_), std::floor(pt.z / downsample_size_)};

                    if(pts_map.find(idx) == pts_map.end())
                    {
                        pts_map[idx] = {pt.vec3(), pt.vec3(), 1};
                    }
                    else
                    {
                        std::tuple<Vec3, Vec3, int>& temp = pts_map[idx];
                        double dist = (pt.vec3() - std::get<0>(temp)).squaredNorm();
                        if(dist < half_voxel_size_sq_)
                        {
                            std::get<1>(temp) = std::get<1>(temp) + pt.vec3();
                            std::get<2>(temp)++;
                        }
                    }
                }
                std::vector<Vec3> downsampled_pts;
                for(const auto& kv: pts_map)
                {
                    downsampled_pts.push_back(std::get<1>(kv.second) / std::get<2>(kv.second));
                }

                // If the number of points is too high, randomly select a subset
                if(downsampled_pts.size() > max_nb_pts_)
                {
                    std::vector<int> indexes = generateRandomIndexes(0, downsampled_pts.size(), max_nb_pts_);
                    std::vector<Vec3> temp_pts;
                    for(int idx: indexes)
                    {
                        temp_pts.push_back(downsampled_pts[idx]);
                    }
                    downsampled_pts = temp_pts;
                }

                sw.stop();
                sw.print("Downsampling time");

                std::cout << "Number of downsampled pts for registration: " << downsampled_pts.size() << std::endl;
                sw.reset();
                sw.start();

                // Register the points
                //Mat4 prior = current_pose_*delta_trans;

                map_mutex_.lock();
                if(!with_prior_)
                {
                    current_pose_ = map_->registerPts(downsampled_pts, prior_, pts[0].t, true, false);
                    prior_ = current_pose_;
                }

                current_pose_ = map_->registerPts(downsampled_pts, prior_, pts[0].t, approximate_);
                prior_ = current_pose_;
                key_framing_time_cumulated_ = 0.0;
                map_mutex_.unlock();

                // Publish the odom to map correction
                Mat4 odom_map_correction = current_pose_ * trans.inverse();
                geometry_msgs::TransformStamped odom_map_correction_msg;
                odom_map_correction_msg.header.stamp = time;
                odom_map_correction_msg.header.frame_id = "map";
                odom_map_correction_msg.child_frame_id = "odom";
                odom_map_correction_msg.transform = mat4ToTransform(odom_map_correction);
                odom_map_correction_pub_.publish(odom_map_correction_msg);

                sw.stop();
                sw.print("Registering time");
            }
            else
            {
                current_pose_ = trans;
            }
            sw.reset();
            sw.start();

            map_mutex_.lock();
            // Add the points to the map
            map_->addPts(pts, current_pose_);

            map_mutex_.unlock();

            sw.stop();
            sw.print("Adding points to map time");

            counter_++;
            last_pc_epoch_time_mutex_.lock();
            last_pc_epoch_time_ = std::chrono::high_resolution_clock::now();
            last_pc_epoch_time_mutex_.unlock();
        }


        void pcPriorCallback(const sensor_msgs::PointCloud2ConstPtr& pc_msg, const geometry_msgs::TransformStampedConstPtr& odom_msg)
        {
            if(first_)
            {
                last_pc_time_ = pc_msg->header.stamp;
            }
            else
            {
                double time_diff = (ros::Time(pc_msg->header.stamp) - ros::Time(last_pc_time_)).toSec();
                if(time_diff < 0)
                {
                    ROS_WARN("Time diff is negative, skipping point cloud");
                    return;
                }
            }
            std::vector<Pointd> pts;
            if(pc_type_internal_)
            {
                pts = pointCloud2MsgToPtsVecInternal(pc_msg);
            }
            else
            {
                bool rubbish0, rubbish1;
                std::tie(pts, rubbish0, rubbish1) = pointCloud2MsgToPtsVec<double>(pc_msg, 1.0, false);
            }
            updateMap(pts, transformToMat4(odom_msg->transform), pc_msg->header.stamp);
            last_pc_time_ = pc_msg->header.stamp;
        }


        void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
        {
            if(first_)
            {
                last_pc_time_ = msg->header.stamp;
            }
            else
            {
                double time_diff = (ros::Time(msg->header.stamp) - ros::Time(last_pc_time_)).toSec();
                if(time_diff < 0)
                {
                    ROS_WARN("Time diff is negative, skipping point cloud");
                    return;
                }
            }
            std::vector<Pointd> pts;
            if (pc_type_internal_)
            {
                pts = pointCloud2MsgToPtsVecInternal(msg);
            }
            else
            {
                bool rubbish0, rubbish1;
                std::tie(pts, rubbish0, rubbish1) = pointCloud2MsgToPtsVec<double>(msg, 1.0, false);
            }
            updateMap(pts, current_pose_, msg->header.stamp);
            last_pc_time_ = msg->header.stamp;
        }
        

        void mapPublishThread()
        {

            while(running_)
            {
                auto start = std::chrono::high_resolution_clock::now();

                int counter = counter_;
                if(counter != previous_counter_)
                {
                    previous_counter_ = counter;
                    if(map_pub_.getNumSubscribers() > 0)
                    {
                        ROS_INFO("Number of points in map: %ld", map_->getPts().size());
                        map_mutex_.lock();
                        std::vector<Pointf> pts = map_->getPts();
                        map_mutex_.unlock();
                        sensor_msgs::PointCloud2 map_msg = ptsVecToPointCloud2MsgInternal(pts, "map", ros::Time::now());
                        map_pub_.publish(map_msg);
                    }
                }
                
                // Check if the last point cloud is too old
                if((map_path_ != "") && (last_write_counter_ != counter))
                {
                    last_pc_epoch_time_mutex_.lock();
                    std::chrono::time_point<std::chrono::high_resolution_clock> last_time_temp = last_pc_epoch_time_;
                    last_pc_epoch_time_mutex_.unlock();
                    if((start-last_time_temp) > std::chrono::duration<double>(2.0*key_framing_time_thr_))
                    {
                        last_write_counter_ = counter;
                        ROS_INFO_STREAM("Writing map to file: " << map_path_);
                        map_mutex_.lock();
                        map_->writeMap(map_path_);
                        map_mutex_.unlock();
                    }
                }

                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = end - start;
                std::this_thread::sleep_for(std::chrono::duration<double>(map_publish_period_) - elapsed);
            }
        }

};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "gp_map");
    GpMapNode node;
    ros::spin();
    return 0;
}



