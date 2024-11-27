#include "rclcpp/rclcpp.hpp"
#include "lice/map_distance_field.h"
#include "lice/ros_utils.h"
#include "lice/utils.h"
#include "lice/math_utils.h"

#include <memory>
#include <thread>
#include <mutex>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "ankerl/unordered_dense.h"

#include "ffastllamaa/srv/query_dist_field.hpp"

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

class GpMapNode: public rclcpp::Node
{
    public:
        GpMapNode()
            : Node("gp_map")
        {
            MapDistFieldOptions options;

            double voxel_size = readRequiredFieldDouble(this, "voxel_size");
            options.cell_size = voxel_size;
            downsample_size_ = readFieldDouble(this, "voxel_size_factor_for_registration", 5.0) * voxel_size;
            half_voxel_size_sq_ = std::pow(voxel_size / 2.0,2);
            options.neighborhood_size = readRequiredFieldInt(this, "neighbourhood_size");

            register_ = readFieldBool(this, "register", true);
            bool with_prior = readRequiredFieldBool(this, "with_prior");
            with_prior_ = with_prior;
            approximate_ = readFieldBool(this, "register_with_approximate_field", false);

            map_publish_period_ = readFieldDouble(this, "map_publish_period", 1.0);

            options.use_temporal_weights = readFieldBool(this, "use_temporal_weights", false);

            options.free_space_carving_radius = readFieldDouble(this, "free_space_carving_radius", -1.0);

            max_nb_pts_ = readFieldInt(this, "max_num_pts_for_registration", 4000);

            options.free_space_carving = false;
            if (options.free_space_carving_radius > 0.0)
            {
                options.free_space_carving = true;
            }
            double min_range = readRequiredFieldDouble(this, "min_range");
            options.min_range = min_range;
            options.max_range = readFieldDouble(this, "max_range", 1000.0);
            min_range_ = min_range;
            max_range_ = options.max_range;

            key_framing_ = readFieldBool(this, "key_framing", false);
            key_framing_dist_thr_ = readFieldDouble(this, "key_framing_dist_thr", 1.0);
            key_framing_rot_thr_ = readFieldDouble(this, "key_framing_rot_thr", 0.1);
            key_framing_time_thr_ = readFieldDouble(this, "key_framing_time_thr", 1.0);

            map_path_ = readFieldString(this, "map_path", "");
            map_path_ = map_path_ + (map_path_.back() == '/' ? "" : "/") + "map.ply";
            // If folder does not exist, create it
            std::string folder = map_path_.substr(0, map_path_.find_last_of("/"));
            if(!folderExists(folder))
            {
                if(!createFolder(folder))
                {
                    RCLCPP_ERROR(this->get_logger(), "Could not create folder: %s for map output", folder.c_str());
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Created folder: %s for map output", folder.c_str());
            }


            options.output_normals = readFieldBool(this, "output_normals", false);
            options.output_mesh = readFieldBool(this, "output_mesh", false);
            options.clean_mesh_threshold = readFieldDouble(this, "clean_mesh_threshold", 2.0*voxel_size);
            options.meshing_point_per_node = readFieldDouble(this, "meshing_point_per_node", 2.0);
            options.poisson_weighted = readFieldBool(this, "poisson_weighted", false);


            pc_type_internal_ = readFieldBool(this, "point_cloud_internal_type", false);

            map_ = std::make_unique<MapDistField>(options);
            


            if(with_prior)
            {
                pc_sub_.subscribe(this, "/points_input");
                pose_sub_.subscribe(this, "/pose_input");
                sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, geometry_msgs::msg::TransformStamped>>(pc_sub_, pose_sub_, 20);
                sync_->registerCallback(std::bind(&GpMapNode::pcPriorCallback, this, std::placeholders::_1, std::placeholders::_2));
            }
            else
            {
                sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points_input", 1, std::bind(&GpMapNode::pcCallback, this, std::placeholders::_1));
            }



            map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map", 10);

            odom_map_correction_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/odom_map_correction", 10);

            map_publish_thread_ = std::make_unique<std::thread>(&GpMapNode::mapPublishThread, this);

            query_dist_field_srv_ = this->create_service<ffastllamaa::srv::QueryDistField>("/query_dist_field", std::bind(&GpMapNode::queryDistFieldCallback, this, std::placeholders::_1, std::placeholders::_2));

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

        std::string map_path_ = "";

        std::unique_ptr<MapDistField> map_;
        std::mutex map_mutex_;

        // Sub for time synchronised prior
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pc_sub_;
        message_filters::Subscriber<geometry_msgs::msg::TransformStamped> pose_sub_;
        std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, geometry_msgs::msg::TransformStamped>> sync_;

        // Sub for no prior
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

        // Global map publisher
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr odom_map_correction_pub_;


        rclcpp::Service<ffastllamaa::srv::QueryDistField>::SharedPtr query_dist_field_srv_;





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
        rclcpp::Time last_pc_time_;
        double key_framing_time_cumulated_ = 0.0;

        double DEBUG_query_time_sum_ = 0.0;
        int DEBUG_query_time_count_ = 0;

        double DEBUG_registration_time_sum_ = 0.0;
        int DEBUG_registration_time_count_ = 0;

        std::unique_ptr<std::thread> map_publish_thread_;

        // Store the last point cloud time
        std::atomic<std::chrono::time_point<std::chrono::high_resolution_clock>> last_pc_epoch_time_;

        void queryDistFieldCallback(const std::shared_ptr<ffastllamaa::srv::QueryDistField::Request> request, std::shared_ptr<ffastllamaa::srv::QueryDistField::Response> response)
        {
            if(request->dim != 3)
            {
                RCLCPP_ERROR(this->get_logger(), "Only 3D points are supported");
                return;
            }
            std::vector<Vec3> query_pts;
            for(size_t i = 0; i < request->num_pts; i++)
            {
                query_pts.push_back(Vec3(request->pts.at(i*3), request->pts.at(i*3+1), request->pts.at(i*3+2)));
            }
            map_mutex_.lock();
            StopWatch sw;
            sw.start();
            std::vector<double> dists = map_->queryDistField(query_pts);
            DEBUG_query_time_sum_ += sw.stop();
            map_mutex_.unlock();
            DEBUG_query_time_count_ += request->num_pts;
            RCLCPP_INFO(this->get_logger(), "Average query time per point (API): %f", DEBUG_query_time_sum_/DEBUG_query_time_count_);
            sw.print("Query time (API) with" + std::to_string(request->num_pts) + " points");
            for(double dist: dists)
            {
                response->dists.push_back(dist);
            }
        }



        void updateMap(const std::vector<Pointd> pts, const Mat4 trans, rclcpp::Time time)
        {
            if(first_)
            {
                last_input_pose_ = trans;
                prior_ = trans;
                first_ = false;

                map_mutex_.lock();
                map_->addPts(pts, current_pose_);
                map_mutex_.unlock();

                geometry_msgs::msg::TransformStamped temp_msg;
                temp_msg.header.stamp = time;
                temp_msg.header.frame_id = "map";
                temp_msg.child_frame_id = "odom";
                temp_msg.transform = mat4ToTransform(Mat4::Identity());
                odom_map_correction_pub_->publish(temp_msg);
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
                    double time_diff = (rclcpp::Time(time) - rclcpp::Time(last_pc_time_)).seconds();
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
                    if( (range < min_range_) || (range > max_range_)) continue;

                    GridIndex idx = {std::floor(pt.x / downsample_size_), std::floor(pt.y / downsample_size_), std::floor(pt.z / downsample_size_)};

                    if(pts_map.find(idx) == pts_map.end())
                    {
                        pts_map[idx] = {pt.vec3(), pt.vec3(), 1};
                    }
                    else
                    {
                        std::tuple<Vec3, Vec3, int>& temp = pts_map[idx];
                        std::get<1>(temp) = std::get<1>(temp) + pt.vec3();
                        std::get<2>(temp)++;
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
                map_mutex_.lock();
                if(!with_prior_)
                {
                    current_pose_ = map_->registerPts(downsampled_pts, current_pose_, pts[0].t, true, false);
                    prior_ = current_pose_;
                }
                sw.start();
                current_pose_ = map_->registerPts(downsampled_pts, prior_, pts[0].t, approximate_);
                prior_ = current_pose_;
                key_framing_time_cumulated_ = 0.0;
                map_mutex_.unlock();

                // Publish the odom to map correction
                Mat4 odom_map_correction = current_pose_ * trans.inverse();
                geometry_msgs::msg::TransformStamped odom_map_correction_msg;
                odom_map_correction_msg.header.stamp = time;
                odom_map_correction_msg.header.frame_id = "map";
                odom_map_correction_msg.child_frame_id = "odom";
                odom_map_correction_msg.transform = mat4ToTransform(odom_map_correction);
                odom_map_correction_pub_->publish(odom_map_correction_msg);

                DEBUG_registration_time_sum_ += sw.stop();
                DEBUG_registration_time_count_++;
                sw.print("Registering time");
                RCLCPP_INFO(this->get_logger(), "------- Average registration time: %f ms", DEBUG_registration_time_sum_/DEBUG_registration_time_count_);
            }
            else
            {
                current_pose_ = trans;
            }

            map_mutex_.lock();
            sw.reset();
            sw.start();
            // Add the points to the map
            map_->addPts(pts, current_pose_);

            map_mutex_.unlock();

            sw.stop();
            sw.print("Adding points to map time");
            counter_++;
            last_pc_epoch_time_ = std::chrono::high_resolution_clock::now();
        }


        void pcPriorCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg, const geometry_msgs::msg::TransformStamped::ConstSharedPtr odom_msg)
        {
            if(first_)
            {
                last_pc_time_ = pc_msg->header.stamp;
            }
            else
            {
                double time_diff = (rclcpp::Time(pc_msg->header.stamp) - rclcpp::Time(last_pc_time_)).seconds();
                if(time_diff < 0)
                {
                    RCLCPP_WARN(this->get_logger(), "Time diff is negative, skipping point cloud");
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
                bool rubish0, rubish1;
                std::tie(pts, rubish0, rubish1) = pointCloud2MsgToPtsVec<double>(pc_msg, 1.0, false);
            }
            updateMap(pts, transformToMat4(odom_msg->transform), pc_msg->header.stamp);
            last_pc_time_ = pc_msg->header.stamp;
        }


        void pcCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            if(first_)
            {
                last_pc_time_ = msg->header.stamp;
            }
            else
            {
                double time_diff = (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_pc_time_)).seconds();
                if(time_diff < 0)
                {
                    RCLCPP_WARN(this->get_logger(), "Time diff is negative, skipping point cloud");
                    return;
                }
            }
            std::vector<Pointd> pts;
            if(pc_type_internal_)
            {
                pts = pointCloud2MsgToPtsVecInternal(msg);
            }
            else
            {
                bool rubish0, rubish1;
                std::tie(pts, rubish0, rubish1) = pointCloud2MsgToPtsVec<double>(msg, 1.0, false);
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
                    if(map_pub_->get_subscription_count() > 0)
                    {
                        RCLCPP_INFO(this->get_logger(), "Publishing map points");
                        map_mutex_.lock();
                        std::vector<Pointf> pts = map_->getPts();
                        map_mutex_.unlock();
                        sensor_msgs::msg::PointCloud2 map_msg = ptsVecToPointCloud2MsgInternal(pts, "map", this->now());
                        map_pub_->publish(map_msg);
                    }
                }

                // Check if the last point cloud is too old
                if((map_path_ != "") && (last_write_counter_ != counter))
                {
                    std::chrono::time_point<std::chrono::high_resolution_clock> last_time_temp = last_pc_epoch_time_;
                    if((start-last_time_temp) > std::chrono::duration<double>(2.0*key_framing_time_thr_))
                    {
                        last_write_counter_ = counter;
                        RCLCPP_INFO_STREAM(this->get_logger(), "Writing map to file: " << map_path_);
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
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



