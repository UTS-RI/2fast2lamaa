#pragma once

#include "rclcpp/rclcpp.hpp"
#include "lice/ros_utils.h"
#include "lice/utils.h"
#include "lice/types.h"
#include "lice/lidar_odometry.h"
#include <memory>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/imu.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


class LidarOdometry;


class LidarOdometryNode : public rclcpp::Node
{
    public:
        LidarOdometryNode();

        void publishTransform(const double t, const Vec3& pos, const Vec3& rot);
        void publishDeltaTransform(const double t, const Vec3& dp, const Vec3& dr);
        void publishPc(const double t, const std::vector<Pointf>& pc);
        void publishPcFiltered(const double t, const std::vector<Pointf>& pc_static, const std::vector<Pointf>& pc_dynamic, const std::vector<Pointf>& pc_unsure);
        void publishGlobalOdom(const double t, const Vec3& pos, const Vec3& rot, const Vec3& vel, const Vec3& ang_vel);

    private:
        std::shared_ptr<LidarOdometry> lidar_odometry_;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr acc_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyr_sub_;
        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr odom_map_correction_sub_;

        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> feature_sub_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pc_sub_;

        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr odom_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr global_odom_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_twist_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;


        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_static_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_dynamic_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_unsure_pub_;

        std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>> sync_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> br_;

        std::mutex mutex_br_;
        std::mutex mutex_pc_;
        std::mutex mutex_pc_filtered_;

        rclcpp::Time first_t_;
        rclcpp::Time first_acc_t_;
        rclcpp::Time first_gyr_t_;
        bool first_ = true;
        bool first_acc_ = true;
        bool first_gyr_ = true;
        int scan_count_ = 0;
        bool invert_imu_ = false;

        bool acc_in_m_s2_ = true;

        geometry_msgs::msg::TransformStamped::SharedPtr odom_map_correction_msg_;


        void pcCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr feature_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg);

        void accCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

        void gyrCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

        void odomMapCorrectionCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
};