#include "lice/lidar_odometry.h"
#include <iostream>
#include <random>
#include "KDTree.h"
#include "lice/dynamic_score.h"

typedef jk::tree::KDTree<std::pair<int,int>, 3, 16> KDTree3;


void LidarOdometry::addPc(const std::shared_ptr<std::vector<Pointd>>& pc, const std::shared_ptr<std::vector<Pointd> > features, const double t)
{
    mutex_.lock();
    pc_.push_back(pc);
    features_.push_back(features);
    pc_t_.push_back(t);
    mutex_.unlock();
}

void LidarOdometry::addAccSample(const Vec3& acc, const double t)
{
    mutex_.lock();
    ugpm::ImuSample imu_sample;
    imu_sample.data[0] = acc[0];
    imu_sample.data[1] = acc[1];
    imu_sample.data[2] = acc[2];
    imu_sample.t = t;
    imu_data_.acc.push_back(imu_sample);
    mutex_.unlock();
}

void LidarOdometry::addGyroSample(const Vec3& gyro, const double t)
{
    mutex_.lock();
    ugpm::ImuSample imu_sample;
    imu_sample.data[0] = gyro[0];
    imu_sample.data[1] = gyro[1];
    imu_sample.data[2] = gyro[2];
    imu_sample.t = t;
    imu_data_.gyr.push_back(imu_sample);
    mutex_.unlock();

}




void LidarOdometry::run()
{
    std::cout << "Starting lidar odometry thread" << std::endl;
    running_ = true;
    while(running_)
    {
        bool run_optimise = false;
        mutex_.lock();
        if(features_.size() >= (size_t)(params_.nb_scans_per_submap))
        {
            run_optimise = true;
            int last_id = params_.nb_scans_per_submap - 1;
            double last_t = pc_[last_id]->back().t;


            // Check if last acc and last gyro timestamps are later than the last point cloud timestamp
            if((imu_data_.acc.size() > 0)&&(imu_data_.gyr.size() > 0))
            {
                if((imu_data_.acc.back().t < last_t)||(imu_data_.gyr.back().t < last_t))
                {
                    run_optimise = false;
                }
            }

        }
        mutex_.unlock();
        if(run_optimise)
        {
            optimise();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(int(kPullPeriod*1000)));
    }
    std::cout << "Stopping lidar odometry thread" << std::endl;
}




std::shared_ptr<std::thread> LidarOdometry::runThread()
{
    return std::make_shared<std::thread>(&LidarOdometry::run, this);
}




void LidarOdometry::stop()
{
    mutex_.lock();
    running_ = false;
    mutex_.unlock();
}




LidarOdometry::LidarOdometry(const LidarOdometryParams& params, LidarOdometryNode* node)
    : params_(params)
    , node_(node)
    , output_slots_(3, true)
{
    std::cout << "LidarOdometry constructor" << std::endl;

    state_period_ = 1.0/params_.state_frequency;
    state_frequency_ = params_.state_frequency;

    state_blocks_.resize(4, Vec3::Zero());

    current_pos_ = Vec3::Zero();
    current_rot_ = Vec3::Zero();

    Vec3 temp;
    temp << params_.calib_rx, params_.calib_ry, params_.calib_rz;
    ceres::AngleAxisToQuaternion<double>(temp.data(), state_calib_.data());

    state_calib_[4] = params_.calib_px;
    state_calib_[5] = params_.calib_py;
    state_calib_[6] = params_.calib_pz;

    loss_function_ = NULL; //new ceres::CauchyLoss(0.05);

    imu_data_.acc_var = params_.acc_std*params_.acc_std;
    imu_data_.gyr_var = params_.gyr_std*params_.gyr_std;

    lidar_weight_ = 1.0/params_.lidar_std;
}




std::vector<std::shared_ptr<std::vector<Pointd> > > LidarOdometry::projectPoints(
        const std::vector<std::shared_ptr<std::vector<Pointd> > >& pts,
        const State& state,
        const std::vector<Vec3>& state_blocks,
        const double time_offset,
        const Vec7& state_calib) const
{
    std::vector<std::shared_ptr<std::vector<Pointd> > > output(pts.size());

    for(size_t i = 0; i < pts.size(); ++i)
    {
        output[i] = std::make_shared<std::vector<Pointd> >();
        output[i]->resize(pts[i]->size());

        Mat3 R_calib;
        ceres::QuaternionToRotation<double>(state_calib.data(), R_calib.data());
        Vec3 t_calib = state_calib.segment<3>(4);

        std::vector<double> temp_times;
        temp_times.reserve(pts[i]->size());
        for(size_t j = 0; j < pts[i]->size(); ++j)
        {
            temp_times.push_back(pts[i]->at(j).t);
        }
        std::vector<std::pair<Vec3, Vec3>> poses = state.query(temp_times, state_blocks[0], state_blocks[1], state_blocks[2], state_blocks[3], time_offset);

        //#pragma omp parallel for
        for(size_t j = 0; j < pts[i]->size(); ++j)
        {
            //auto [pos, rot] = state.query(pts[i]->at(j).t, state_blocks[0], state_blocks[1], state_blocks[2], state_blocks[3], time_offset);
            Vec3& pos = poses[j].first;
            Vec3& rot = poses[j].second;

            Vec3 p_L = pts[i]->at(j).vec3();
            Vec3 p_I = R_calib * p_L + t_calib;
            Vec3 p_W;
            ceres::AngleAxisRotatePoint<double>(rot.data(), p_I.data(), p_W.data());
            p_W += pos;

            output[i]->at(j) = Pointd(p_W, pts[i]->at(j).t, pts[i]->at(j).i, pts[i]->at(j).channel, pts[i]->at(j).type, pts[i]->at(j).scan_id, pts[i]->at(j).dynamic);

        }
    }
    return output;
}


std::vector<std::shared_ptr<std::vector<Pointd> > > LidarOdometry::projectPoints(
        const std::vector<std::shared_ptr<std::vector<Pointd> > >& pts,
        const Vec3& pos,
        const Vec3& rot)
{
    std::vector<std::shared_ptr<std::vector<Pointd> > > output(pts.size());

    for(size_t i = 0; i < pts.size(); ++i)
    {
        output[i] = std::make_shared<std::vector<Pointd> >();
        output[i]->resize(pts[i]->size());
        #pragma omp parallel for
        for(size_t j = 0; j < pts[i]->size(); ++j)
        {
            Vec3 p_L = pts[i]->at(j).vec3();
            Vec3 p_W;
            ceres::AngleAxisRotatePoint<double>(rot.data(), p_L.data(), p_W.data());
            p_W += pos;

            output[i]->at(j) = Pointd(p_W, pts[i]->at(j).t, pts[i]->at(j).i, pts[i]->at(j).channel, pts[i]->at(j).type, pts[i]->at(j).scan_id, pts[i]->at(j).dynamic);

        }
    }
    return output;
}







void LidarOdometry::printState()
{
    std::cout << "State: " << std::endl;
    std::cout << "    acc_bias: " << state_blocks_[0].transpose() << std::endl;
    std::cout << "    gyr_bias: " << state_blocks_[1].transpose() << std::endl;
    std::cout << "    gravity: " << state_blocks_[2].transpose() << std::endl;
    std::cout << "    vel: " << state_blocks_[3].transpose() << std::endl;
    std::cout << "    time_offset: " << time_offset_ << std::endl;
    std::cout << "    calib: " << state_calib_.transpose() << std::endl;
}






std::vector<DataAssociation> LidarOdometry::findDataAssociations(
    const std::set<int>& types,
    const std::shared_ptr<std::vector<Pointd> >& features,
    const int pc_id,
    const std::vector<std::shared_ptr<std::vector<Pointd> > >& targets,
    const std::vector<int>& target_ids)
{

    std::vector<DataAssociation> data_associations;
    double max_dist2 = params_.max_feature_dist*params_.max_feature_dist;

    // Create the kd tree for each feature type
    std::vector<std::shared_ptr<KDTree3>> feature_kd_trees;
    std::map<int, int> tree_types;
    int counter = 0;
    for(const auto type: types)
    {
        tree_types[type] = counter;
        feature_kd_trees.push_back(std::make_shared<KDTree3>());
        counter++;
    }
    for(size_t i = 0; i < targets.size(); ++i)
    {
        for(size_t j = 0; j < targets[i]->size(); ++j)
        {
            if(targets[i]->at(j).type != 3)
            {
                Vec3f p = targets[i]->at(j).vec3f();
                feature_kd_trees[tree_types[targets[i]->at(j).type]]->addPoint({p[0], p[1], p[2]}, {i, j});
            }
        }
    }

                    

    std::vector<bool> valid = std::vector<bool>(features->size(), false);
    std::vector<DataAssociation> association_candidates(features->size());

    for(size_t i = 0; i < features->size(); ++i)
    {
        int type = features->at(i).type;
        if(feature_kd_trees[tree_types[type]]->size() == 0)
            continue;

        int tree_id = tree_types[type];
        KDTree3& tree = *(feature_kd_trees.at(tree_id));
        Vec3f temp_feature = features->at(i).vec3f();


        // Plannar Look for the 3 closest point, making sure they are not aligned
        if(type == 1)
        {
            auto nn = tree.searchCapacityLimitedBall({temp_feature(0), temp_feature(1), temp_feature(2)}, max_dist2, 6);

            if(nn.size() < 3)
                continue;


            int target_id = nn[0].payload.first;
            int target_feature_id = nn[0].payload.second;

            Vec3f candidate_1 = targets[target_id]->at(target_feature_id).vec3f();
            int candidate_2_id = 1;
            // Get the second cadidate with at distance greater that params_.min_feature_dist between the first and the second


            while(((size_t)(candidate_2_id) < nn.size())&&
                ((candidate_1 - targets[nn[candidate_2_id].payload.first]->at(nn[candidate_2_id].payload.second).vec3f()).norm() < params_.min_feature_dist))
            {
                candidate_2_id++;
            }

            if((size_t)(candidate_2_id) >= nn.size())
                continue;


            int candidate_3_id = candidate_2_id + 1;
            Vec3f candidate_2 = targets[nn[candidate_2_id].payload.first]->at(nn[candidate_2_id].payload.second).vec3f();

            Vec3f v1 = candidate_2 - candidate_1;
            v1.normalize();
            // Get the third candidate with at distance greater that params_.min_feature_dist between the the first, the second and the third, also check that the three points are not aligned
            while(((size_t)(candidate_3_id) < nn.size())&&
                ((candidate_1 - targets[nn[candidate_3_id].payload.first]->at(nn[candidate_3_id].payload.second).vec3f()).norm() < params_.min_feature_dist)&&
                ((candidate_2 - targets[nn[candidate_3_id].payload.first]->at(nn[candidate_3_id].payload.second).vec3f()).norm() < params_.min_feature_dist)&&
                (std::abs(v1.dot((targets[nn[candidate_3_id].payload.first]->at(nn[candidate_3_id].payload.second).vec3f() - candidate_2).normalized())) > 0.4))
            {
                candidate_3_id++;
            }

            if((size_t)(candidate_3_id) < nn.size())
            {
                DataAssociation data_association;
                data_association.pc_id = pc_id;
                data_association.feature_id = i;
                data_association.type = type;
                data_association.target_ids.push_back(std::make_pair(target_ids[nn[0].payload.first], nn[0].payload.second));
                data_association.target_ids.push_back(std::make_pair(target_ids[nn[candidate_2_id].payload.first], nn[candidate_2_id].payload.second));
                data_association.target_ids.push_back(std::make_pair(target_ids[nn[candidate_3_id].payload.first], nn[candidate_3_id].payload.second));
                valid[i] = true;
                association_candidates[i] = data_association;
            }

        }
        else if((type == 2))
        {
            auto nn = tree.searchCapacityLimitedBall({temp_feature(0), temp_feature(1), temp_feature(2)}, max_dist2, 5);

            if(nn.size() < 2) 
                continue;


            int target_id = nn[0].payload.first;
            int target_feature_id = nn[0].payload.second;

            Vec3f candidate_1 = targets[target_id]->at(target_feature_id).vec3f();
            int candidate_2_id = 1; 
            // Get the second cadidate with at distance greater that params_.min_feature_dist between t  he first and the second


            while(((size_t)(candidate_2_id) < nn.size())&&
                ((candidate_1 - targets[nn[candidate_2_id].payload.first]->at(nn[candidate_2_id].payload.second).vec3f()).norm() < params_.min_feature_dist))
            {
                candidate_2_id++;
            }

            if((size_t)(candidate_2_id) < nn.size())
            {
                DataAssociation data_association;
                data_association.pc_id = pc_id;
                data_association.feature_id = i; 
                data_association.type = type;
                data_association.target_ids.push_back(std::make_pair(target_ids[nn[0].payload.first], nn[0].payload.second));
                data_association.target_ids.push_back(std::make_pair(target_ids[nn[candidate_2_id].payload.first], nn[candidate_2_id].payload.second));
                valid[i] = true;
                association_candidates[i] = data_association;
            }
        }
    }


    for(size_t i = 0; i < features->size(); ++i)
    {
        if(valid[i])
        {
            data_associations.push_back(association_candidates[i]);
        }
    }

    return data_associations;
}




void LidarOdometry::visualiseDataAssociation(const std::vector<DataAssociation>& data_association)
{
    // Visualise the data association with cilantro
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3f> colors;
    std::vector<Eigen::Vector3f> pc_colors;
    pc_colors.push_back(Eigen::Vector3f(0.0, 0.0, 0.0));
    pc_colors.push_back(Eigen::Vector3f(1.0, 0.0, 0.0));
    pc_colors.push_back(Eigen::Vector3f(0.0, 1.0, 0.0));
    pc_colors.push_back(Eigen::Vector3f(0.0, 0.0, 1.0));
    std::vector<Eigen::Vector3f> target_colors;
    target_colors.push_back(Eigen::Vector3f(0.0, 0.0, 0.0));
    target_colors.push_back(Eigen::Vector3f(0.5, 0.0, 0.0));
    target_colors.push_back(Eigen::Vector3f(0.0, 0.5, 0.0));
    target_colors.push_back(Eigen::Vector3f(0.0, 0.0, 0.5));
    std::vector<Eigen::Vector3f> points_asso_1;
    std::vector<Eigen::Vector3f> points_asso_2;
    for(size_t i = 0; i < data_association.size(); ++i)
    {
        points.push_back(features_[data_association[i].pc_id]->at(data_association[i].feature_id).vec3f());
        if((size_t)(data_association[i].type) < pc_colors.size())
            colors.push_back(Eigen::Vector3f(1.0, 0.0, 0.0));
        else
            colors.push_back(Eigen::Vector3f(0.5, 0.5, 0.5));

        for(size_t j = 0; j < data_association[i].target_ids.size(); ++j)
        {
            points_asso_1.push_back(features_[data_association[i].target_ids[j].first]->at(data_association[i].target_ids[j].second).vec3f());
            points_asso_2.push_back(features_[data_association[i].pc_id]->at(data_association[i].feature_id).vec3f());

            points.push_back(features_[data_association[i].target_ids[j].first]->at(data_association[i].target_ids[j].second).vec3f());
            if((size_t)(data_association[i].type) < target_colors.size())
                colors.push_back(target_colors[data_association[i].type]);
            else
                colors.push_back(Eigen::Vector3f(0.5, 0.5, 0.5));
        }
    }

    //cilantro::PointCloud3f points_cloud;
    //points_cloud.points.resize(3,points.size()); 
    //points_cloud.colors.resize(3,points.size());
    //for(int i = 0; i < points.size(); ++i)
    //{
    //    points_cloud.points.col(i) = points[i];
    //    points_cloud.colors.col(i) = colors[i];
    //}

    //pangolin::CreateWindowAndBind(window_name, 640, 480);
    //cilantro::Visualizer viz(window_name, "disp1");
    //viz.addObject<cilantro::PointCloudRenderable>("points", points_cloud);
    //viz.addObject<cilantro::PointCorrespondencesRenderable>("correspondences", points_asso_1, points_asso_2);


    //while(!viz.wasStopped())
    //{
    //    viz.spinOnce();
    //}
    





}




void LidarOdometry::visualisePoints(const std::vector<std::shared_ptr<std::vector<Pointd> > >& points)
{
    std::vector<Eigen::Vector3f> points_vec;
    for(size_t i = 0; i < points.size(); ++i)
    {
        for(size_t j = 0; j < points[i]->size(); ++j)
        {
            points_vec.push_back(points[i]->at(j).vec3f());
        }
    }

    //cilantro::PointCloud3f points_cloud;
    //points_cloud.points.resize(3,points_vec.size());
    //for(int i = 0; i < points_vec.size(); ++i)
    //{
    //    points_cloud.points.col(i) = points_vec[i];
    //}

    //pangolin::CreateWindowAndBind(window_name, 640, 480);
    //cilantro::Visualizer viz(window_name, "disp1");
    //viz.addObject<cilantro::PointCloudRenderable>("points", points_cloud);

    //// Add axis
    //viz.addObject<cilantro::CoordinateFrameRenderable>("axis", Eigen::Matrix4f::Identity(), 1.0f,
    //  cilantro::RenderingProperties().setLineWidth(5.0f));
    //while(!viz.wasStopped())
    //{
    //    viz.spinOnce();
    //}

}




std::tuple<std::vector<std::shared_ptr<std::vector<Pointd> > >, ugpm::ImuData> LidarOdometry::getDataForOptimisation()
{
    // Copy the data to let the other threads continue
    mutex_.lock();
    std::vector<std::shared_ptr<std::vector<Pointd> > > features;
    for(int i = 0; i < params_.nb_scans_per_submap; ++i)
    {
        features.push_back(features_[i]);
    }
    ugpm::ImuData imu_data = imu_data_.get(pc_t_[0] - state_period_, std::max(imu_data_.acc.back().t, imu_data_.gyr.back().t));
    mutex_.unlock();

    return {features, imu_data};
}




void LidarOdometry::initState(const ugpm::ImuData& imu_data)
{
    // Create the state times and the K_inv matrix
    Vec3 acc_temp;
    acc_temp[0] = imu_data.acc[0].data[0];
    acc_temp[1] = imu_data.acc[0].data[1];
    acc_temp[2] = imu_data.acc[0].data[2];

    acc_temp.normalize();
    acc_temp *= -params_.g;

    state_blocks_[2] = acc_temp;
}




std::vector<DataAssociation> LidarOdometry::getAllAssociations(
        const std::set<int>& types
        , const std::vector<std::shared_ptr<std::vector<Pointd> > >& pts)
{

    std::vector<DataAssociation> data_associations;
    {
        std::vector<int> target_ids(1, 0);
        std::vector<std::shared_ptr<std::vector<Pointd> > > target_features(1, pts[0]);
        std::vector<DataAssociation> temp_data_associations = findDataAssociations(types, pts.back(), pts.size()-1, target_features, target_ids);

        data_associations_save_.push_back(temp_data_associations);

        data_associations.insert(data_associations.end(), temp_data_associations.begin(), temp_data_associations.end());

    }
    //// If first scan, look for associations from 0, if not, look from 1
    //int start_id = 0;
    //std::cout << " Get all associations, first_optimisation_ " << first_optimisation_ << std::endl;
    //if(!first_optimisation_)
    //{
    //    start_id = pts.size()-2;
    //}
    //else
    //{
    //    for(int i = 0; i < data_associations_save_.size(); ++i)
    //    {
    //        for(int j = 0; j < data_associations_save_[i].size(); ++j)
    //        {
    //            data_associations.push_back(data_associations_save_[i][j]);
    //        }
    //        
    //    }
    //}
    //for(int i = start_id; i < pts.size()-1; ++i)
    //{
    //    std::cout << "Get all associations, new associations i " << i << std::endl;
    //    std::vector<int> target_ids(1, i+1);
    //    std::vector<std::shared_ptr<std::vector<Point> > > target_features(1, pts[i+1]);
    //    std::vector<DataAssociation> temp_data_associations = findDataAssociations(types, pts[i], i, target_features, target_ids);

    //    data_associations_save_.push_back(temp_data_associations);

    //    data_associations.insert(data_associations.end(), temp_data_associations.begin(), temp_data_associations.end());

    //}

    //if(!first_optimisation_)
    //{
    //    std::cout << "Removing first entry in data_associations_save_" << std::endl;
    //    std::cout << "data_associations_save_ size " << data_associations_save_.size() << std::endl;

    //    // Remove the first entry in data_associations_save_
    //    if(data_associations_save_.size() > 1)
    //    {
    //        data_associations_save_.erase(data_associations_save_.begin());
    //    }
    //    for(auto& v_asso : data_associations_save_)
    //    {
    //        for(auto& asso : v_asso)
    //        {
    //            // Remove 1 to all the pc_ids
    //            asso.pc_id--;
    //            for(int i = 0; i < asso.target_ids.size(); ++i)
    //            {
    //                asso.target_ids[i].first--;
    //            }
    //        }
    //    }
    //}
    //else
    //{
    //    data_associations_save_.clear();
    //}


    return data_associations;
}





std::vector<DataAssociation> LidarOdometry::filterDataAssociations(const std::vector<DataAssociation>& association_in, const std::vector<std::shared_ptr<std::vector<Pointd> > >& pts)
{
    // For each data associations get the normal vector or the line vector
    std::map<int, std::map<std::tuple<int, int, int, int, int>, std::vector<DataAssociation> > > association_maps;
    for(size_t i = 0; i < association_in.size(); ++i)
    {
        int type = association_in[i].type;

        if(association_maps.count(type) == 0)
        {
            association_maps[type] = std::map<std::tuple<int, int, int, int, int>, std::vector<DataAssociation> >();
        }

        if(association_in[i].target_ids.size() == 3)
        {
            std::pair<int, int> target_id = association_in[i].target_ids[0];
            Vec3 target_1 = pts[target_id.first]->at(target_id.second).vec3();
            target_id = association_in[i].target_ids[1];
            Vec3 target_2 = pts[target_id.first]->at(target_id.second).vec3();
            target_id = association_in[i].target_ids[2];
            Vec3 target_3 = pts[target_id.first]->at(target_id.second).vec3();

            Vec3 center = pts[association_in[i].pc_id]->at(association_in[i].feature_id).vec3();

            Vec3 v1 = target_2 - target_1;
            Vec3 v2 = target_3 - target_1;
            Vec3 normal = v1.cross(v2);
            normal.normalize();

            // Get the elevation and azimuth
            double elevation = std::asin(normal[2]);
            double azimuth = std::atan2(normal[1], normal[0]);

            int azimuth_id = int(azimuth/kAssociationFilterAngQuantum);
            int elevation_id = int(elevation/kAssociationFilterAngQuantum);
            int x_id = int(center[0]/kAssociationFilterLinQuantum);
            int y_id = int(center[1]/kAssociationFilterLinQuantum);
            int z_id = int(center[2]/kAssociationFilterLinQuantum);

        
            //std::cout << "IDs " << azimuth_id << "  " << elevation_id << "  " << x_id << "  " << y_id << "  " << z_id << std::endl;
            
            if(association_maps[type].count(std::make_tuple(azimuth_id, elevation_id, x_id, y_id, z_id)) > 0)
            {
                association_maps[type][std::make_tuple(azimuth_id, elevation_id, x_id, y_id, z_id)].push_back(association_in[i]);
            }
            else
            {
                association_maps[type][std::make_tuple(azimuth_id, elevation_id, x_id, y_id, z_id)] = std::vector<DataAssociation>(1, association_in[i]);
            }

        }
        else if(association_in[i].target_ids.size() == 2)
        {
            std::pair<int, int> target_id = association_in[i].target_ids[0];
            Vec3 target_1 = pts[target_id.first]->at(target_id.second).vec3();
            target_id = association_in[i].target_ids[1];
            Vec3 target_2 = pts[target_id.first]->at(target_id.second).vec3();

            Vec3 v1 = target_2 - target_1;
            v1.normalize();
            Vec3 center = pts[association_in[i].pc_id]->at(association_in[i].feature_id).vec3();

            // Get the elevation and azimuth
            double elevation = std::asin(v1[2]);
            double azimuth = std::atan2(v1[1], v1[0]);

            int azimuth_id = int(azimuth/kAssociationFilterAngQuantum);
            int elevation_id = int(elevation/kAssociationFilterAngQuantum);
            int x_id = int(center[0]/kAssociationFilterLinQuantum);
            int y_id = int(center[1]/kAssociationFilterLinQuantum);
            int z_id = int(center[2]/kAssociationFilterLinQuantum);
            
            if(association_maps[type].count(std::make_tuple(azimuth_id, elevation_id, x_id, y_id, z_id)) > 0)
            {
                association_maps[type][std::make_tuple(azimuth_id, elevation_id, x_id, y_id, z_id)].push_back(association_in[i]);
            }
            else
            {
                association_maps[type][std::make_tuple(azimuth_id, elevation_id, x_id, y_id, z_id)] = std::vector<DataAssociation>(1, association_in[i]);
            }
        }
        else
        {
            throw std::runtime_error("LidarOdometry::filterDataAssociation: Unknown/non-implemented number of target ids");
        }

    }

    // For each association map
    std::vector<std::vector<DataAssociation>> association_out;
    std::vector<int> association_counters;
    for(auto& [type, association_map] : association_maps)
    {
        association_counters.push_back(0);
        association_out.push_back(std::vector<DataAssociation>());
        for(auto& [key, associations] : association_map)
        {
            // Pick randomly up to kMaxNbAssociationPerBin associations
            auto rng = std::default_random_engine {};
            std::shuffle(associations.begin(), associations.end(), rng);
            int nb_associations = std::min(kMaxNbAssociationPerBin, int(associations.size()));
            int temp_asso_size = associations.size();
            for(int i = temp_asso_size - 1; i >= temp_asso_size - nb_associations; --i)
            {
                association_out.back().push_back(associations.back());
                associations.pop_back();
                association_counters.back()++;
            }
        }
        std::cout << "Nb associations for type " << type << " before capping: " << association_counters.back() << std::endl;

        if(association_counters.back() > kMaxNbAssociationsPerType)
        {
            auto rng = std::default_random_engine {};
            std::shuffle(association_out.back().begin(), association_out.back().end(), rng);
            association_out.back().resize(kMaxNbAssociationsPerType);
            association_counters.back() = kMaxNbAssociationsPerType;
        }
    }
    
    std::vector<DataAssociation> association_out_flat;
    for(size_t i = 0; i < association_out.size(); ++i)
    {
        association_out_flat.insert(association_out_flat.end(), association_out[i].begin(), association_out[i].end());
    }



    
    return association_out_flat;
}






void LidarOdometry::addBlocks(ceres::Problem& problem)
{
    // Create the SE3 manifold
    ceres::ProductManifold<ceres::QuaternionManifold, ceres::EuclideanManifold<3>>* se3 = new ceres::ProductManifold<ceres::QuaternionManifold, ceres::EuclideanManifold<3>>();

    // Add the state variables
    problem.AddParameterBlock(state_blocks_[0].data(), 3);
    problem.AddParameterBlock(state_blocks_[1].data(), 3);
    ceres::SphereManifold<3>* sphere = new ceres::SphereManifold<3>();
    problem.AddParameterBlock(state_blocks_[2].data(), 3, sphere);

    ZeroPrior* prior = new ZeroPrior(3, 100);
    problem.AddResidualBlock(prior, NULL, state_blocks_[0].data());
    problem.AddParameterBlock(state_blocks_[3].data(), 3);
    problem.AddParameterBlock(state_calib_.data(), 7, se3);
    problem.AddParameterBlock(&time_offset_, 1);

    // Debug
    problem.SetParameterBlockConstant(&time_offset_);
    problem.SetParameterBlockConstant(state_calib_.data());
}



void LidarOdometry::addLidarResiduals(ceres::Problem& problem
        , const std::vector<DataAssociation>& data_associations
        , const std::vector<std::shared_ptr<std::vector<Pointd> > >& pts
        , const State& state
        )
{
    // Add the residuals
    std::cout << "Adding " << data_associations.size() << " lidar residuals" << std::endl;


    //LidarAllCostFunction* cost_function = new LidarAllCostFunction(state, data_associations, pts);
    //problem.AddResidualBlock(cost_function, loss_function_, state_blocks_[0].data(), state_blocks_[1].data(), state_blocks_[2].data(), state_blocks_[3].data(), &time_offset_, state_calib_.data());

    for(size_t i = 0; i < data_associations.size(); ++i)
    {
        LidarCostFunction* cost_function = new LidarCostFunction(state, data_associations[i], pts, lidar_weight_);

        problem.AddResidualBlock(cost_function, loss_function_, state_blocks_[0].data(), state_blocks_[1].data(), state_blocks_[2].data(), state_blocks_[3].data(), &time_offset_, state_calib_.data());
    }

}




void LidarOdometry::createProblemAssociateAndOptimise(
        const std::vector<std::shared_ptr<std::vector<Pointd> > >& pts
        , const State& state
        , const std::set<int>& types
        , const int nb_iter)
{
    // Perform the optimisation
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = nb_iter;
    options.num_threads = 16;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.function_tolerance = 1e-4;
    //options.check_gradients = true;


    StopWatch sw;
    sw.start();

    // Project the features to the state times
    std::vector<std::shared_ptr<std::vector<Pointd> > > projected_features = projectPoints(
        pts,
        state,
        state_blocks_,
        time_offset_,
        state_calib_);
    sw.stop();
    sw.print("Projection: ");
    
    sw.reset();
    sw.start();
    // Scan to scan correspondences
    std::vector<DataAssociation> data_associations = getAllAssociations(types, projected_features);

    //visualiseDataAssociation(data_associations, "Data association "+ std::to_string(ros::Time::now().toSec()) +"  " + std::to_string(pc_t_[0]) + " scan to scan");

    sw.stop();
    sw.print("Data association: ");
    std::cout << "Nb data associations: " << data_associations.size() << std::endl;

    sw.reset();
    sw.start();
    data_associations = filterDataAssociations(data_associations, projected_features);
    sw.stop();
    sw.print("Association filtering");
    std::cout << "Nb data associations after filtering: " << data_associations.size() << std::endl;

    //visualiseDataAssociation(data_associations, "Data association after filtering "+ std::to_string(ros::Time::now().toSec()) +"  " + std::to_string(pc_t_[0]) + " scan to scan");

    ceres::Problem::Options pb_options;
    pb_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

    ceres::Problem problem(pb_options);
    addBlocks(problem);
    addLidarResiduals(problem, data_associations, pts, state);


    // Solve the problem
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;


}



void LidarOdometry::visualiseRawSubmap()
{
    std::vector<std::shared_ptr<std::vector<Pointd> > > temp_pc;
    for(int i = 0; i < params_.nb_scans_per_submap; ++i)
    {
        temp_pc.push_back(pc_[i]);
    }
    visualisePoints(temp_pc);
}

void LidarOdometry::visualiseSubmap(const State& state)
{
    std::vector<std::shared_ptr<std::vector<Pointd> > > temp_pc;
    for(int i = 0; i < params_.nb_scans_per_submap; ++i)
    {
        temp_pc.push_back(pc_[i]);
    }
    std::vector<std::shared_ptr<std::vector<Pointd> > > projected_pc = projectPoints(
        temp_pc,
        state,
        state_blocks_,
        time_offset_,
        state_calib_);
    visualisePoints(projected_pc);
}



void LidarOdometry::prepareSubmap(const State state, std::vector<std::shared_ptr<std::vector<Pointd> > > temp_pc, std::vector<double> temp_pc_t, std::vector<Vec3> state_blocks, Vec7 state_calib, double time_offset)
{

    StopWatch sw;
    sw.start();

    
    int mid_id = params_.id_scan_to_publish; //int(params_.nb_scans_per_submap/2);
    double mid_t = temp_pc_t.at(mid_id);

    node_->publishTransform(mid_t, current_pos_, current_rot_);

    // Compute all the scan poses that are needed
    auto [mid_pos, mid_rot] = state.query(mid_t, state_blocks.at(0), state_blocks.at(1), state_blocks.at(2), state_blocks.at(3), time_offset);

    auto [inv_mid_pos, inv_mid_rot] = invertTransform(mid_pos, mid_rot);

    double temp_next_time;
    if(mid_id < params_.nb_scans_per_submap - 1)
    {
        temp_next_time = temp_pc_t[mid_id+1];
    }
    else
    {
        temp_next_time = temp_pc[mid_id]->back().t;
    }
    auto [next_pos, next_rot] = state.query(temp_next_time, state_blocks_[0], state_blocks_[1], state_blocks_[2], state_blocks_[3], time_offset_);

    auto [increment_pos, increment_rot] = combineTransforms(inv_mid_pos, inv_mid_rot, next_pos, next_rot);

    std::tie(current_pos_, current_rot_) = combineTransforms(current_pos_, current_rot_, increment_pos, increment_rot);

    // Publish the odometry at the end of the scan
    auto [twist_linear, twist_angular] = state.queryTwist(temp_next_time, state_blocks_[0], state_blocks_[1], state_blocks_[2], state_blocks_[3], time_offset_);

    node_->publishGlobalOdom(temp_next_time, current_pos_, current_rot_, twist_linear, twist_angular);


    node_->publishDeltaTransform(mid_t, increment_pos, increment_rot);

    mutex_output_.lock();
    int slot_id = -1;
    for(size_t i = 0; i < output_slots_.size(); ++i)
    {
        if(output_slots_[i])
        {
            output_slots_[i] = false;
            slot_id = i;
            break;
        }
    }
    mutex_output_.unlock();
    if(slot_id < 0)
    {
        std::cout << "WARNING: No available slot for output (dynamic filtering might be too slow, consider keyframing)" << std::endl;
        return;
    }

    
    std::vector<int> start_ids;
    start_ids.push_back(0);
    int nb_pts = 0;

    if(params_.publish_all_scans || params_.dynamic_filtering)
    {
        for(int i = 0; i < params_.nb_scans_per_submap; ++i)
        {
            start_ids.push_back(start_ids.back() + temp_pc.at(i)->size());
            nb_pts += temp_pc.at(i)->size();
        }
    }
    else
    {
        nb_pts = temp_pc.at(mid_id)->size();
    }



    std::vector<Pointf>projected_pc(nb_pts);


    Mat3 R_calib;
    ceres::QuaternionToRotation<double>(state_calib.data(), R_calib.data());
    Vec3 p_calib = state_calib.segment<3>(4);

    Mat3 R_mid = ugpm::expMap(mid_rot);
    Vec3 p_mid = mid_pos;

    int start_id = (params_.publish_all_scans || params_.dynamic_filtering ? 0 : mid_id);
    int end_id = (params_.publish_all_scans || params_.dynamic_filtering ? params_.nb_scans_per_submap : mid_id+1);
    for(int i = start_id; i < end_id; ++i)
    {
        std::vector<double> local_temp_pc_t;
        local_temp_pc_t.reserve(temp_pc.at(i)->size());
        for(size_t j = 0; j < temp_pc.at(i)->size(); ++j)
        {
            local_temp_pc_t.push_back(temp_pc.at(i)->at(j).t);
        }


        std::vector<std::pair<Vec3, Vec3> > poses = state.queryApprox(local_temp_pc_t, state_blocks[0], state_blocks[1], state_blocks[2], state_blocks[3], time_offset);


        for(size_t j = 0; j < temp_pc.at(i)->size(); ++j)
        {

            Vec3& pos = poses[j].first;
            Vec3& rot = poses[j].second;

            Vec3 p_L = temp_pc.at(i)->at(j).vec3();
            Vec3 p_I = R_calib * p_L + p_calib;
            Vec3 p_W;
            ceres::AngleAxisRotatePoint<double>(rot.data(), p_I.data(), p_W.data());
            p_W += pos;

            p_W = R_mid.transpose() * (p_W - p_mid);

            projected_pc.at(start_ids[i-start_id]+j) = Pointf(float(p_W[0]), float(p_W[1]), float(p_W[2]), temp_pc[i]->at(j).t, temp_pc[i]->at(j).i, temp_pc[i]->at(j).channel, temp_pc[i]->at(j).type, temp_pc[i]->at(j).scan_id, temp_pc[i]->at(j).dynamic);

        }
    }

    if(params_.dynamic_filtering && (!params_.publish_all_scans))
    {
        auto first = projected_pc.begin() + start_ids[mid_id];
        auto last = first + temp_pc.at(mid_id)->size();
        std::vector<Pointf> temp_pc(first, last);
        node_->publishPc(mid_t, temp_pc);
    }
    else
    {
        node_->publishPc(mid_t, projected_pc);
    }
    
    bool compute = false;
    if(params_.key_framing)
    {
        double time_diff = mid_t - last_output_time_;
        Mat4 current_pose = posRotToTransform(current_pos_, current_rot_);
        if(time_diff > params_.key_frame_time)
        {
            last_output_time_ = mid_t;
            last_output_pose_ = current_pose;
            compute = true;
        }
        if(!compute)
        {
            auto [dist, angle] = distanceBetweenTransforms(current_pose, last_output_pose_);
            if(dist > params_.key_frame_dist || angle > params_.key_frame_angle)
            {
                last_output_time_ = mid_t;
                last_output_pose_ = current_pose;
                compute = true;
            }
        }
    }
    else
    {
        compute = true;
    }

    if(params_.dynamic_filtering && compute)
    {
        auto [static_pc, dynamic_pc, unsure_pc] = dynamicFiltering(projected_pc, params_.dynamic_filtering_threshold, params_.dynamic_filtering_voxel_size);
        node_->publishPcFiltered(mid_t, static_pc, dynamic_pc, unsure_pc);
    }

    mutex_output_.lock();
    output_slots_[slot_id] = true;
    mutex_output_.unlock();

    sw.stop();
    sw.print("Prepare submap for later publishing: ");
}


void LidarOdometry::prepareNextState(const State& state)
{
    mutex_.lock();
    auto [next_pos, next_rot] = state.query(pc_t_[1], state_blocks_[0], state_blocks_[1], state_blocks_[2], state_blocks_[3], time_offset_);

    Mat3 R = ugpm::expMap(next_rot);

    state_blocks_[0] = Vec3::Zero();
    state_blocks_[1] = Vec3::Zero();

    state_blocks_[2] = R.transpose()*state_blocks_[2];
    state_blocks_[3] = R.transpose()*state_blocks_[3];


    // Remove the first point cloud and features
    pc_.erase(pc_.begin());
    features_.erase(features_.begin());
    pc_t_.erase(pc_t_.begin());

    imu_data_ = imu_data_.get(pc_t_[0]- 2*state_period_, std::numeric_limits<double>::max());
    mutex_.unlock();

}




void LidarOdometry::optimise()
{   
    std::cout << "LidarOdometry::optimise" << std::endl;

    StopWatch sw;
    sw.start();

    // Get the data for optimisation
    auto [features, imu_data] = getDataForOptimisation();

    sw.stop();
    sw.print("Get data for optimisation: ");

    std::cout << "Optimizing with " << features.size() << " scans, " << imu_data_.acc.size() << " acc samples, and " << imu_data_.gyr.size() << " gyr samples" << std::endl;

    sw.reset();
    sw.start();

    double submap_t = pc_t_.at(0);
    double last_t = pc_[params_.nb_scans_per_submap-1]->back().t;

    State state(imu_data, submap_t, last_t, state_frequency_);

    // Initialise the state values
    std::cout << "IMU initialisation not implemented yet" << std::endl;

    if(first_optimisation_)
    {
        initState(imu_data_);
    }

    
    sw.stop();
    sw.print("State precomputations: ");


    sw.reset();
    sw.start();

    // Create the problem and optimise
    if(first_optimisation_)
    {
        createProblemAssociateAndOptimise(features, state, kTypes, 5);
        createProblemAssociateAndOptimise(features, state, kTypes, 5);
    }
    createProblemAssociateAndOptimise(features, state, kTypes, 14);



    // Launch the preparation of the submap in a separate thread
    mutex_.lock();
    std::vector<std::shared_ptr<std::vector<Pointd> > > temp_pc;
    std::vector<double> temp_pc_t;
    for(int i = 0; i < params_.nb_scans_per_submap; ++i)
    {
        temp_pc.push_back(pc_.at(i));
        temp_pc_t.push_back(pc_t_.at(i));
    }
    
    mutex_.unlock();

    std::thread submap_thread(&LidarOdometry::prepareSubmap, this, state, temp_pc, temp_pc_t, state_blocks_, state_calib_, time_offset_);
    submap_thread.detach();


    prepareNextState(state);

    sw.stop();
    sw.print("Optimisation and launch publisher: ");

    
    first_optimisation_ = false;
}