#pragma once

#include "types.h"
#include <mutex>
#include <thread>
#include "preint/preint.h"
#include "lice/utils.h"
#include "lice/cost_functions.h"
#include "lice/state.h"
#include "lice/lidar_odometry_node.h"


#include <ceres/ceres.h>
#include <ceres/rotation.h>

class LidarOdometryNode;

const double kPullPeriod = 0.01;
const double kAssociationFilterAngQuantum = 1.0;
const double kAssociationFilterLinQuantum = 0.75;

const int kMaxNbAssociationPerBin = 3;
const int kMaxNbAssociationsPerType = 500;

struct LidarOdometryParams
{
    double min_feature_dist = 0.05;
    double max_feature_dist = 0.5;
    int nb_scans_per_submap = 2;
    int id_scan_to_publish = 1;
    double state_frequency = 10;
    double g = 9.80;
    double calib_px = 0.0;
    double calib_py = 0.0;
    double calib_pz = 0.0;
    double calib_rx = 0.0;
    double calib_ry = 0.0;
    double calib_rz = 0.0;

    double gyr_std = 0.005;
    double acc_std = 0.02;
    double lidar_std = 0.02;

    bool publish_all_scans = false;

    bool dynamic_filtering = false;
    float dynamic_filtering_threshold = 0.5;
    float dynamic_filtering_voxel_size = 0.15;

    bool key_framing = false;
    double key_frame_dist = 1.0;
    double key_frame_angle = 0.25;
    double key_frame_time = 0.5;

};





class LidarOdometry
{

    public:

        LidarOdometry(const LidarOdometryParams& params, LidarOdometryNode* node);

        void addPc(const std::shared_ptr<std::vector<Pointd>>& pc, const std::shared_ptr<std::vector<Pointd> > features, const double t);

        void addAccSample(const Vec3& acc, const double t);
        void addGyroSample(const Vec3& gyro, const double t);

        void stop();
        void run();
        std::shared_ptr<std::thread> runThread();


        // Destructor
        ~LidarOdometry()
        {
            delete loss_function_;
        }

    private:

        LidarOdometryParams params_;
        LidarOdometryNode* node_;

        std::mutex mutex_;

        std::vector<std::shared_ptr<std::vector<Pointd> > > pc_;
        std::vector<std::shared_ptr<std::vector<Pointd> > > features_;
        std::vector<double> pc_t_;
        std::vector<std::vector<DataAssociation> > data_associations_save_;

        std::mutex mutex_output_;
        std::vector<bool> output_slots_;
        double last_output_time_ = std::numeric_limits<double>::lowest();
        Mat4 last_output_pose_ = Mat4::Identity();


        double lidar_weight_ = 1.0;


        Vec3 current_pos_ = Vec3::Zero();
        Vec3 current_rot_ = Vec3::Zero();



        bool first_acc_ = true;
        bool first_optimisation_ = true;
        //ros::Time first_t_;

        bool running_ = false;


        ugpm::ImuData imu_data_;
        // acc_bias, gyr_bias, gravity, vel
        std::vector<Vec3> state_blocks_;
        double time_offset_ = 0.0;
        Vec7 state_calib_;

        double state_period_;
        double state_frequency_;

        ceres::LossFunction* loss_function_;

        void optimise();

        std::vector<std::shared_ptr<std::vector<Pointd> > > projectPoints(
            const std::vector<std::shared_ptr<std::vector<Pointd> > >& pts,
            const State& state,
            const std::vector<Vec3>& state_blocks,
            const double time_offset,
            const Vec7& state_calib) const;

        std::vector<std::shared_ptr<std::vector<Pointd> > > projectPoints(
            const std::vector<std::shared_ptr<std::vector<Pointd> > >& pts,
            const Vec3& pos,
            const Vec3& rot);

        std::vector<DataAssociation> findDataAssociations(
            const std::set<int>& types,
            const std::shared_ptr<std::vector<Pointd> >& features,
            const int pc_id,
            const std::vector<std::shared_ptr<std::vector<Pointd> > >& tragets,
            const std::vector<int>& target_ids);


        std::tuple<std::vector<std::shared_ptr<std::vector<Pointd> > >, ugpm::ImuData> getDataForOptimisation();

        void initState(const ugpm::ImuData& imu_data);

        void addBlocks(ceres::Problem& problem);

        void addBlockPriors(ceres::Problem& problem);

        std::vector<DataAssociation> getAllAssociations(
                const std::set<int>& types
                , const std::vector<std::shared_ptr<std::vector<Pointd> > >& pts);

        std::vector<DataAssociation> filterDataAssociations(const std::vector<DataAssociation>& association_in, const std::vector<std::shared_ptr<std::vector<Pointd> > >& pts);

        void addLidarResiduals(
                ceres::Problem& problem
                , const std::vector<DataAssociation>& data_associations
                , const std::vector<std::shared_ptr<std::vector<Pointd> > >& pts
                , const State& state
                );
        

        void createProblemAssociateAndOptimise(
                const std::vector<std::shared_ptr<std::vector<Pointd> > >& pts
                , const State& state
                , const std::set<int>& types
                , const int nb_iter=50);


        void printState();

        void prepareSubmap(const State state, const std::vector<std::shared_ptr<std::vector<Pointd> > > pcs, const std::vector<double> pcs_t, std::vector<Vec3> state_blocks, Vec7 state_calib, double time_offset);

        void prepareNextState(const State& state);

        // For DEBUG
        void visualiseDataAssociation(const std::vector<DataAssociation>& data_association);
        void visualisePoints(const std::vector<std::shared_ptr<std::vector<Pointd> > >& pts);

        void visualiseRawSubmap();
        void visualiseSubmap(const State& state);


};


