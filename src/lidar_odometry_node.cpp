#include "lice/lidar_odometry_node.h"



LidarOdometryNode::LidarOdometryNode()
    : rclcpp::Node("lidar_odometry")
{
    RCLCPP_INFO(this->get_logger(), "Starting lidar_odometry node");

    LidarOdometryParams params;
    params.min_feature_dist = readFieldDouble(this, "min_feature_dist", 0.05);
    params.max_feature_dist = readFieldDouble(this, "max_feature_dist", 0.5);
    params.nb_scans_per_submap = readFieldInt(this, "nb_scans_per_submap", 2);
    params.id_scan_to_publish = readFieldInt(this, "id_scan_to_publish", 1);
    if (params.id_scan_to_publish < 0 || params.id_scan_to_publish >= params.nb_scans_per_submap)
    {
        RCLCPP_ERROR(this->get_logger(), "id_scan_to_publish must be between 0 and nb_scans_per_submap-1");
        throw std::invalid_argument("Invalid parameter");
    }
    params.state_frequency = readFieldDouble(this, "state_freq", 100.0);
    acc_in_m_s2_ = readFieldBool(this, "acc_in_m_per_s2", true);
    invert_imu_ = readFieldBool(this, "invert_imu", true);

    params.calib_px = readRequiredFieldDouble(this, "calib_px");
    params.calib_py = readRequiredFieldDouble(this, "calib_py");
    params.calib_pz = readRequiredFieldDouble(this, "calib_pz");
    params.calib_rx = readRequiredFieldDouble(this, "calib_rx");
    params.calib_ry = readRequiredFieldDouble(this, "calib_ry");
    params.calib_rz = readRequiredFieldDouble(this, "calib_rz");

    params.publish_all_scans = readFieldBool(this, "publish_all_scans", false);
    params.dynamic_filtering = readFieldBool(this, "dynamic_filtering", false);
    params.dynamic_filtering_threshold = float(readFieldDouble(this, "dynamic_filtering_threshold", 0.5));
    params.dynamic_filtering_voxel_size = float(readFieldDouble(this, "dynamic_filtering_voxel_size", 0.15));
    params.key_framing = readFieldBool(this, "key_framing", false);
    params.key_frame_dist = readFieldDouble(this, "key_frame_dist_thr", 1.0);
    params.key_frame_angle = readFieldDouble(this, "key_frame_rot_thr", 0.25);
    params.key_frame_time = readFieldDouble(this, "key_frame_time_thr", 0.1);



    lidar_odometry_ = std::make_shared<LidarOdometry>(params, this);


    odom_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/undistortion_delta_transform", 10);
    global_odom_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/undistortion_pose", 10);
    odom_twist_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/end_of_scan_odom", 10);
    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_scan_undistorted", 10);

    if(params.dynamic_filtering)
    {
        pc_static_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_static", 10);
        pc_dynamic_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_dynamic", 10);
        pc_unsure_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_unsure", 10);
    }

    acc_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/acc", 100, std::bind(&LidarOdometryNode::accCallback, this, std::placeholders::_1));
    gyr_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/gyr", 100, std::bind(&LidarOdometryNode::gyrCallback, this, std::placeholders::_1));

    odom_map_correction_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/odom_map_correction", 10, std::bind(&LidarOdometryNode::odomMapCorrectionCallback, this, std::placeholders::_1));

    feature_sub_.subscribe(this, "/lidar_features");
    pc_sub_.subscribe(this, "/lidar_scan");
    sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>>(feature_sub_, pc_sub_, 20);
    sync_->registerCallback(std::bind(&LidarOdometryNode::pcCallback, this, std::placeholders::_1, std::placeholders::_2));

    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_handle = this->get_node_topics_interface();
    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this, tf2_ros::DynamicBroadcasterQoS(), rclcpp::PublisherOptions());

    auto thread = lidar_odometry_->runThread();


    rclcpp::spin(this->get_node_base_interface());

    lidar_odometry_->stop();
    thread->join();

}


void LidarOdometryNode::publishTransform(const double t, const Vec3& pos, const Vec3& rot)
{
    rclcpp::Time new_time = first_t_ + rclcpp::Duration::from_seconds(t);

    // Send a TF transform
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = new_time;
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "lidar";
    transformStamped.transform.translation.x = pos[0];
    transformStamped.transform.translation.y = pos[1];
    transformStamped.transform.translation.z = pos[2];

    Eigen::AngleAxisd aa = Eigen::AngleAxisd(rot.norm(), rot.normalized());
    Eigen::Quaterniond q(aa);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    //static tf2_ros::TransformBroadcaster br;
    mutex_br_.lock();
    if(odom_map_correction_msg_)
    {
        auto temp_msg = *odom_map_correction_msg_;
        temp_msg.header.stamp = new_time;
        br_->sendTransform(temp_msg);
    }
    br_->sendTransform(transformStamped);
    global_odom_pub_->publish(transformStamped);
    mutex_br_.unlock();
}
void LidarOdometryNode::publishDeltaTransform(const double t, const Vec3& dp, const Vec3& dr)
{
    rclcpp::Time new_time = first_t_ + rclcpp::Duration::from_seconds(t);

    // Send the delta transform
    geometry_msgs::msg::TransformStamped delta_trans;
    delta_trans.header.stamp = new_time;
    delta_trans.header.frame_id = "lidar";
    delta_trans.child_frame_id = "lidar_head";
    delta_trans.transform.translation.x = dp[0];
    delta_trans.transform.translation.y = dp[1];
    delta_trans.transform.translation.z = dp[2];

    Eigen::AngleAxisd aa2 = Eigen::AngleAxisd(dr.norm(), dr.normalized());
    Eigen::Quaterniond q2(aa2);
    delta_trans.transform.rotation.x = q2.x();
    delta_trans.transform.rotation.y = q2.y();
    delta_trans.transform.rotation.z = q2.z();
    delta_trans.transform.rotation.w = q2.w();

    // Publish the delta transform
    mutex_br_.lock();
    br_->sendTransform(delta_trans);
    odom_pub_->publish(delta_trans);
    mutex_br_.unlock();
}
void LidarOdometryNode::publishGlobalOdom(const double t, const Vec3& pos, const Vec3& rot, const Vec3& vel, const Vec3& ang_vel)
{
    rclcpp::Time new_time = first_t_ + rclcpp::Duration::from_seconds(t);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = new_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "lidar";
    odom_msg.pose.pose.position.x = pos[0];
    odom_msg.pose.pose.position.y = pos[1];
    odom_msg.pose.pose.position.z = pos[2];

    Eigen::AngleAxisd aa = Eigen::AngleAxisd(rot.norm(), rot.normalized());
    Eigen::Quaterniond q(aa);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = vel[0];
    odom_msg.twist.twist.linear.y = vel[1];
    odom_msg.twist.twist.linear.z = vel[2];

    odom_msg.twist.twist.angular.x = ang_vel[0];
    odom_msg.twist.twist.angular.y = ang_vel[1];
    odom_msg.twist.twist.angular.z = ang_vel[2];

    for (size_t i = 0; i < 36; ++i)
    {
        if (i % 6 == 0)
        {
            odom_msg.pose.covariance[i] = 1;
            odom_msg.twist.covariance[i] = 1;
        }
        else
        {
            odom_msg.pose.covariance[i] = 0.0;
            odom_msg.twist.covariance[i] = 0.0;
        }
    }

    odom_twist_pub_->publish(odom_msg);

}

void LidarOdometryNode::publishPc(const double t, const std::vector<Pointf>& pc)
{
    rclcpp::Time new_time = first_t_ + rclcpp::Duration::from_seconds(t);
    sensor_msgs::msg::PointCloud2 pc_msg = ptsVecToPointCloud2MsgInternal(pc, "lidar", new_time);
    mutex_pc_.lock();
    pc_pub_->publish(pc_msg);
    mutex_pc_.unlock();
}
void LidarOdometryNode::publishPcFiltered(const double t, const std::vector<Pointf>& pc_static, const std::vector<Pointf>& pc_dynamic, const std::vector<Pointf>& pc_unsure)
{
    rclcpp::Time new_time = first_t_ + rclcpp::Duration::from_seconds(t);

    sensor_msgs::msg::PointCloud2 static_msg = ptsVecToPointCloud2MsgInternal(pc_static, "lidar", new_time);
    sensor_msgs::msg::PointCloud2 dynamic_msg = ptsVecToPointCloud2MsgInternal(pc_dynamic, "lidar", new_time);
    sensor_msgs::msg::PointCloud2 unsure_msg = ptsVecToPointCloud2MsgInternal(pc_unsure, "lidar", new_time);
    mutex_pc_filtered_.lock();
    pc_static_pub_->publish(static_msg);
    pc_dynamic_pub_->publish(dynamic_msg);
    pc_unsure_pub_->publish(unsure_msg);
    mutex_pc_filtered_.unlock();
}


void LidarOdometryNode::odomMapCorrectionCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    mutex_br_.lock();
    // Save the correction
    odom_map_correction_msg_ = msg;
    mutex_br_.unlock();
}


void LidarOdometryNode::pcCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr feature_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg)
{
    rclcpp::Time header_time = feature_msg->header.stamp;
    if(first_)
    {
        first_ = false;
        first_t_ = feature_msg->header.stamp;
    }
    if(first_acc_ || first_gyr_)
    {
        return;
    }
    if( (header_time < first_acc_t_) || (header_time < first_gyr_t_) )
    {
        return;
    }

    // Read the point clouds
    std::vector<Pointd> features = pointCloud2MsgToPtsVecInternal(feature_msg);
    std::vector<Pointd> pts = pointCloud2MsgToPtsVecInternal(pc_msg);
    for(size_t i = 0; i < pts.size(); ++i)
    {
        pts[i].scan_id = scan_count_;
    }
    for(size_t i = 0; i < features.size(); ++i)
    {
        features[i].scan_id = scan_count_;
    }
    scan_count_++;

    double time_offset = (header_time-first_t_).seconds();

    // Create shared pointers to the point clouds
    std::shared_ptr<std::vector<Pointd> > features_ptr = std::make_shared<std::vector<Pointd> >(std::move(features));
    std::shared_ptr<std::vector<Pointd> > pts_ptr = std::make_shared<std::vector<Pointd> >(std::move(pts));

    for(size_t i = 0; i < pts_ptr->size(); ++i)
    {
        pts_ptr->at(i).t += time_offset;
    }
    for(size_t i = 0; i < features_ptr->size(); ++i)
    {
        features_ptr->at(i).t += time_offset;
    }

    lidar_odometry_->addPc(pts_ptr, features_ptr, time_offset);

}

void LidarOdometryNode::accCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    rclcpp::Time header_time = msg->header.stamp;
    if(first_)
    {
        first_ = false;
        first_t_ = header_time;
    }
    if(first_acc_)
    {
        first_acc_ = false;
        first_acc_t_ = header_time;
    }

    if(header_time <= last_acc_time_)
    {
        return;
    }
    last_acc_time_ = header_time;

    Vec3 acc;
    acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    if(!acc_in_m_s2_)
    {
        acc *= 9.81;
    }
    if (invert_imu_)
    {
        acc *= -1;
    }
    lidar_odometry_->addAccSample(acc, (header_time-first_t_).seconds());
}

void LidarOdometryNode::gyrCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    rclcpp::Time header_time = msg->header.stamp;
    if(first_)
    {
        first_ = false;
        first_t_ = header_time;
    }
    if(first_gyr_)
    {
        first_gyr_ = false;
        first_gyr_t_ = header_time;
    }

    if(header_time <= last_gyr_time_)
    {
        return;
    }
    last_gyr_time_ = header_time;

    Vec3 gyr;
    gyr << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    if (invert_imu_)
    {
        gyr *= -1;
    }
    lidar_odometry_->addGyroSample(gyr, (header_time-first_t_).seconds());
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarOdometryNode>();
    rclcpp::shutdown();
    return 0;
}

