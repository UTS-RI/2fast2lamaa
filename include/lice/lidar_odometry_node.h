#pragma once

#include "ros/ros.h"
#include "lice/ros_utils.h"
#include "lice/utils.h"
#include "lice/types.h"
#include "lice/lidar_odometry.h"
#include <memory>

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


class LidarOdometry;


class LidarOdometryNode
{
    public:
        LidarOdometryNode();

        void publishTransform(const double t, const Vec3& pos, const Vec3& rot);
        void publishDeltaTransform(const double t, const Vec3& dp, const Vec3& dr);
        void publishPc(const double t, const std::vector<Pointf>& pc);
        void publishPcFiltered(const double t, const std::vector<Pointf>& pc_static, const std::vector<Pointf>& pc_dynamic, const std::vector<Pointf>& pc_unsure);

    private:
        std::shared_ptr<LidarOdometry> lidar_odometry_;

        ros::Subscriber acc_sub_;
        ros::Subscriber gyr_sub_;
        ros::Subscriber odom_map_correction_sub_;

        ros::Publisher odom_pub_;
        ros::Publisher global_odom_pub_;
        ros::Publisher pc_pub_;


        ros::Publisher pc_static_pub_;
        ros::Publisher pc_dynamic_pub_;
        ros::Publisher pc_unsure_pub_;

        std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>> sync_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> br_;

        std::mutex mutex_br_;
        std::mutex mutex_pc_;
        std::mutex mutex_pc_filtered_;

        ros::Time first_t_;
        ros::Time first_acc_t_;
        ros::Time first_gyr_t_;
        bool first_ = true;
        bool first_acc_ = true;
        bool first_gyr_ = true;
        int scan_count_ = 0;
        bool invert_imu_ = false;

        bool acc_in_m_s2_ = true;

        geometry_msgs::TransformStampedConstPtr odom_map_correction_msg_;


        void pcCallback(const sensor_msgs::PointCloud2ConstPtr& feature_msg, const sensor_msgs::PointCloud2ConstPtr& pc_msg);

        void accCallback(const sensor_msgs::ImuConstPtr& msg);

        void gyrCallback(const sensor_msgs::ImuConstPtr& msg);

        void odomMapCorrectionCallback(const geometry_msgs::TransformStampedConstPtr& msg);
};