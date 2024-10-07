#include "lice/lidar_odometry_node.h"



LidarOdometryNode::LidarOdometryNode()
{
    ros::NodeHandle nh("~");

    ROS_INFO("Starting lidar_odometry node");

    LidarOdometryParams params;
    params.min_feature_dist = readField<double>(nh, "min_feature_dist", 0.05);
    params.max_feature_dist = readField<double>(nh, "max_feature_dist", 0.5);
    params.nb_scans_per_submap = readField<int>(nh, "nb_scans_per_submap", 2);
    params.state_frequency = readField<double>(nh, "state_freq", 100.0);
    acc_in_m_s2_ = readField<bool>(nh, "acc_in_m_per_s2", true);
    invert_imu_ = readField<bool>(nh, "invert_imu", true);

    params.calib_px = readRequiredField<double>(nh, "calib_px");
    params.calib_py = readRequiredField<double>(nh, "calib_py");
    params.calib_pz = readRequiredField<double>(nh, "calib_pz");
    params.calib_rx = readRequiredField<double>(nh, "calib_rx");
    params.calib_ry = readRequiredField<double>(nh, "calib_ry");
    params.calib_rz = readRequiredField<double>(nh, "calib_rz");

    params.publish_all_scans = readField<bool>(nh, "publish_all_scans", false);
    params.dynamic_filtering = readField<bool>(nh, "dynamic_filtering", false);
    params.dynamic_filtering_threshold = float(readField<double>(nh, "dynamic_filtering_threshold", 0.5));
    params.dynamic_filtering_voxel_size = float(readField<double>(nh, "dynamic_filtering_voxel_size", 0.15));
    params.key_framing = readField<bool>(nh, "key_framing", false);
    params.key_frame_dist = readField<double>(nh, "key_frame_dist_thr", 1.0);
    params.key_frame_angle = readField<double>(nh, "key_frame_rot_thr", 0.25);
    params.key_frame_time = readField<double>(nh, "key_frame_time_thr", 0.1);



    lidar_odometry_ = std::make_shared<LidarOdometry>(params, this);

    odom_pub_ = nh.advertise<geometry_msgs::TransformStamped>("/undistortion_delta_transform", 10);
    global_odom_pub_ = nh.advertise<geometry_msgs::TransformStamped>("/undistortion_pose", 10);
    pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/lidar_scan_undistorted", 10);

    if(params.dynamic_filtering)
    {
        pc_static_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/lidar_static", 10);
        pc_dynamic_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/lidar_dynamic", 10);
        pc_unsure_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/lidar_unsure", 10);
    }

    acc_sub_ = nh.subscribe("/imu/acc", 100, &LidarOdometryNode::accCallback, this);
    gyr_sub_ = nh.subscribe("/imu/gyr", 100, &LidarOdometryNode::gyrCallback, this);

    odom_map_correction_sub_ = nh.subscribe("/odom_map_correction", 10, &LidarOdometryNode::odomMapCorrectionCallback, this);

    message_filters::Subscriber<sensor_msgs::PointCloud2> feature_sub(nh, "/lidar_features", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "/lidar_scan", 10);
    sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>>(feature_sub, pc_sub, 20);
    sync_->registerCallback(std::bind(&LidarOdometryNode::pcCallback, this, std::placeholders::_1, std::placeholders::_2));

    br_ = std::make_unique<tf2_ros::TransformBroadcaster>();

    auto thread = lidar_odometry_->runThread();

    ros::spin();

    lidar_odometry_->stop();
    thread->join();

}


void LidarOdometryNode::publishTransform(const double t, const Vec3& pos, const Vec3& rot)
{
    ros::Time new_time = first_t_ + ros::Duration(t);

    // Send a TF transform
    geometry_msgs::TransformStamped transformStamped;
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
    global_odom_pub_.publish(transformStamped);
    mutex_br_.unlock();
}
void LidarOdometryNode::publishDeltaTransform(const double t, const Vec3& dp, const Vec3& dr)
{
    ros::Time new_time = first_t_ + ros::Duration(t);

    // Send the delta transform
    geometry_msgs::TransformStamped delta_trans;
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
    odom_pub_.publish(delta_trans);
    mutex_br_.unlock();
}

void LidarOdometryNode::publishPc(const double t, const std::vector<Pointf>& pc)
{
    ros::Time new_time = first_t_ + ros::Duration(t);
    sensor_msgs::PointCloud2 pc_msg = ptsVecToPointCloud2MsgInternal(pc, "lidar", new_time);
    mutex_pc_.lock();
    pc_pub_.publish(pc_msg);
    mutex_pc_.unlock();
}
void LidarOdometryNode::publishPcFiltered(const double t, const std::vector<Pointf>& pc_static, const std::vector<Pointf>& pc_dynamic, const std::vector<Pointf>& pc_unsure)
{
    ros::Time new_time = first_t_ + ros::Duration(t);

    sensor_msgs::PointCloud2 static_msg = ptsVecToPointCloud2MsgInternal(pc_static, "lidar", new_time);
    sensor_msgs::PointCloud2 dynamic_msg = ptsVecToPointCloud2MsgInternal(pc_dynamic, "lidar", new_time);
    sensor_msgs::PointCloud2 unsure_msg = ptsVecToPointCloud2MsgInternal(pc_unsure, "lidar", new_time);
    mutex_pc_filtered_.lock();
    pc_static_pub_.publish(static_msg);
    pc_dynamic_pub_.publish(dynamic_msg);
    pc_unsure_pub_.publish(unsure_msg);
    mutex_pc_filtered_.unlock();
}


void LidarOdometryNode::odomMapCorrectionCallback(const geometry_msgs::TransformStampedConstPtr& msg)
{
    mutex_br_.lock();
    // Save the correction
    odom_map_correction_msg_ = msg;
    mutex_br_.unlock();
}


void LidarOdometryNode::pcCallback(const sensor_msgs::PointCloud2ConstPtr& feature_msg, const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
    ros::Time header_time = feature_msg->header.stamp;
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

    double time_offset = (header_time-first_t_).toSec();

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

void LidarOdometryNode::accCallback(const sensor_msgs::ImuConstPtr& msg)
{
    ros::Time header_time = msg->header.stamp;
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
    lidar_odometry_->addAccSample(acc, (header_time-first_t_).toSec());
}

void LidarOdometryNode::gyrCallback(const sensor_msgs::ImuConstPtr& msg)
{
    ros::Time header_time = msg->header.stamp;
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

    Vec3 gyr;
    gyr << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    if (invert_imu_)
    {
        gyr *= -1;
    }
    lidar_odometry_->addGyroSample(gyr, (header_time-first_t_).toSec());
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_odometry");
    LidarOdometryNode node;
    return 0;
}

