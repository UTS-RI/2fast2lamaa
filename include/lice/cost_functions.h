#pragma once

#include "lice/types.h"
#include "lice/utils.h"
#include "lice/math_utils.h"
#include <ceres/rotation.h>

#include "lice/state.h"



class ZeroPrior : public ceres::CostFunction
{
    private:
        int nb_dim_;
        double weight_;

    public:
        ZeroPrior(const int nb_dim, const double weight): nb_dim_(nb_dim), weight_(weight)
        {
            set_num_residuals(nb_dim);
            mutable_parameter_block_sizes()->push_back(nb_dim);
        }

        bool Evaluate(const double* const* parameters, double* residuals, double** jacobians) const
        {
            Eigen::Map<const VecX> state(parameters[0], nb_dim_);
            Eigen::Map<VecX> res(residuals, nb_dim_);
            res = weight_ * state;

            if(jacobians != NULL)
            {
                if(jacobians[0] != NULL)
                {
                    Eigen::Map<MatX> jac(jacobians[0], nb_dim_, nb_dim_);
                    jac.setIdentity();
                    jac *= weight_;
                }
            }

            return true;
        }
};

class GaussianPrior : public ceres::CostFunction
{
    private:
        int nb_dim_;
        const VecX mean_;
        MatX weight_;

    public:
        GaussianPrior(int nb_dim, const VecX& mean, const MatX& cov): nb_dim_(nb_dim), mean_(mean)
        {
            set_num_residuals(nb_dim);
            mutable_parameter_block_sizes()->push_back(nb_dim);

            MatX cov_inv = cov.inverse();
            Eigen::LLT<MatX> lltOfA(cov);
            weight_ = lltOfA.matrixL().transpose();
        }

        bool Evaluate(const double* const* parameters, double* residuals, double** jacobians) const
        {
            Eigen::Map<const VecX> state(parameters[0], nb_dim_);
            Eigen::Map<VecX> res(residuals, nb_dim_);
            res = weight_*(state-mean_);

            if(jacobians != NULL)
            {
                if(jacobians[0] != NULL)
                {
                    Eigen::Map<MatX> jac(jacobians[0], nb_dim_, nb_dim_);
                    jac = weight_;
                }
            }

            return true;
        }
};


class LidarCostFunction: public ceres::SizedCostFunction<1, 3,3,3,3,1,7>
{
    private:
        const State& state_;
        const DataAssociation data_association_;
        const std::vector<std::shared_ptr<std::vector<Pointd> > >& features_;
        const std::vector<int> block_sizes_;
        std::vector<double> feature_times_;
        const double weight_ = 1.0;


    public:
        LidarCostFunction(
                const State& state
                , const DataAssociation& data_association
                , const std::vector<std::shared_ptr<std::vector<Pointd> > >& features
                , const double weight)
                : state_(state)
                , data_association_(data_association)
                , features_(features)
                , block_sizes_({3,3,3,3,1,7})
                , weight_(weight)
        {
            feature_times_.resize(1+data_association_.target_ids.size());
            feature_times_[0] = features_[data_association_.pc_id]->at(data_association_.feature_id).t;
            for(size_t i = 0; i < data_association_.target_ids.size(); ++i)
            {
                feature_times_[i+1] = features_[data_association_.target_ids[i].first]->at(data_association_.target_ids[i].second).t;
            }
        }

        bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            Eigen::Map<const Vec3> arg_0(parameters[0]);
            Eigen::Map<const Vec3> arg_1(parameters[1]);
            Eigen::Map<const Vec3> arg_2(parameters[2]);
            Eigen::Map<const Vec3> arg_3(parameters[3]);
            double time_offset = parameters[4][0];
            Eigen::Map<const Vec4> quat_I_L(parameters[5]);
            Eigen::Map<const Vec3> pos_I_L(parameters[5]+4);


            std::vector<std::pair<Vec3, Vec3> > poses(1+data_association_.target_ids.size());
            std::vector<std::vector<std::pair<MatX, MatX> > > pose_jacobians(1+data_association_.target_ids.size());
            if(jacobians != NULL)
            {
                //std::tie(poses, pose_jacobians) = state_.queryWthJacobian(feature_times_, arg_0, arg_1, arg_2, arg_3, time_offset);
                for(size_t i = 0; i < feature_times_.size(); ++i)
                {
                    std::tie(poses[i], pose_jacobians[i]) = state_.queryWthJacobian(feature_times_[i], arg_0, arg_1, arg_2, arg_3, time_offset);
                }
            }
            else
            {
                for(size_t i = 0; i < feature_times_.size(); ++i)
                {
                    poses[i] = state_.query(feature_times_[i], arg_0, arg_1, arg_2, arg_3, time_offset);
                }
            }

            Vec3 feature_L = features_[data_association_.pc_id]->at(data_association_.feature_id).vec3();
            Vec3 feature_I;
            ceres::UnitQuaternionRotatePoint(quat_I_L.data(), feature_L.data(), feature_I.data());
            feature_I += pos_I_L;
            Vec3& feature_rot = poses[0].second;
            Vec3& feature_pos = poses[0].first;

            Vec3 feature_W_rot;
            ceres::AngleAxisRotatePoint(feature_rot.data(), feature_I.data(), feature_W_rot.data());
            Vec3 feature_W = feature_W_rot + feature_pos;

            std::vector<Vec3> target_rots(data_association_.target_ids.size());
            std::vector<Vec3> targets_W_rot(data_association_.target_ids.size());
            std::vector<Vec3> targets_W(data_association_.target_ids.size());
            for(size_t j = 0; j < data_association_.target_ids.size(); ++j)
            {
                Vec3 target_L = features_[data_association_.target_ids[j].first]->at(data_association_.target_ids[j].second).vec3();
                Vec3 target_I;
                ceres::UnitQuaternionRotatePoint(quat_I_L.data(), target_L.data(), target_I.data());
                target_I += pos_I_L;
                Vec3& target_rot = poses[j+1].second;
                Vec3& target_pos = poses[j+1].first;

                Vec3 target_W;
                ceres::AngleAxisRotatePoint(target_rot.data(), target_I.data(), target_W.data());
                target_rots[j] = target_rot;
                targets_W_rot[j] = target_W;
                targets_W[j] = target_W + target_pos;
            }


            residuals[0] = weight_ * data_association_.computeResidual(feature_W, targets_W);

            if(jacobians != NULL)
            {
                RowX d_res_d_pts = data_association_.computeJacobian(feature_W, targets_W);
                Mat3 d_feature_d_rot = -ugpm::toSkewSymMat(feature_W_rot)*ugpm::jacobianRighthandSO3(-feature_rot);
                std::vector<Mat3> d_target_d_rot(data_association_.target_ids.size());
                for(size_t j = 0; j < data_association_.target_ids.size(); ++j)
                {
                    Vec3& target_rot = poses[j+1].second;
                    d_target_d_rot[j] = -ugpm::toSkewSymMat(targets_W_rot[j])*ugpm::jacobianRighthandSO3(-target_rot);
                }


                for(int j = 0; j < 5; ++j)
                {
                    if(jacobians[j] != NULL)
                    {
                        int block_size = block_sizes_[j];
                        Eigen::Map<Eigen::Matrix<double, 1,Eigen::Dynamic> > j_s(&(jacobians[j][0]),1,block_size);
                        j_s.setZero();
                        if(pose_jacobians[0][j].first.size() > 0)
                        {
                            j_s += d_res_d_pts.segment<3>(0) * pose_jacobians[0][j].first;
                        }
                        if(pose_jacobians[0][j].second.size() > 0)
                        {
                            j_s += d_res_d_pts.segment<3>(0) * d_feature_d_rot * pose_jacobians[0][j].second;
                        }
                        for(size_t k = 0; k < data_association_.target_ids.size(); ++k)
                        {
                            if(pose_jacobians[k+1][j].first.size() > 0)
                            {
                                j_s += d_res_d_pts.segment<3>(3*(k+1)) * pose_jacobians[k+1][j].first;
                            }
                            if(pose_jacobians[k+1][j].second.size() > 0)
                            {
                                j_s += d_res_d_pts.segment<3>(3*(k+1)) * d_target_d_rot[k] * pose_jacobians[k+1][j].second;
                            }
                        }
                        j_s *= weight_;
                    }
                }

                if(jacobians[5] != NULL)
                {
                    Mat3 R_W_I = ugpm::expMap(feature_rot);
                    std::vector<Mat3> target_R_W_I(data_association_.target_ids.size());
                    for(size_t k = 0; k < data_association_.target_ids.size(); ++k)
                    {
                        target_R_W_I[k] = ugpm::expMap(target_rots[k]);
                    }

                    Eigen::Map<Eigen::Matrix<double, 1, 7> > j_s(&(jacobians[5][0]));
                    j_s.setOnes();
                    MatX d_pts_d_s(3+(3*data_association_.target_ids.size()), 7);
                    d_pts_d_s.block<3,4>(0,0) = R_W_I*jacobianQuatRotation(quat_I_L, feature_L);
                    d_pts_d_s.block<3,3>(0,4) = R_W_I;
                    for(size_t k = 0; k < data_association_.target_ids.size(); ++k)
                    {
                        d_pts_d_s.block<3,4>(3*(k+1),0) = target_R_W_I[k]*jacobianQuatRotation(quat_I_L, features_[data_association_.target_ids[k].first]->at(data_association_.target_ids[k].second).vec3());
                        d_pts_d_s.block<3,3>(3*(k+1),4) = target_R_W_I[k];
                    }

                    j_s = weight_ * d_res_d_pts * d_pts_d_s; 
                }
            }

            return true;
        }
};

//class LidarAllCostFunction: public ceres::CostFunction
//{
//    private:
//        const State& state_;
//        std::vector<std::vector<double> > feature_times_;
//        const std::vector<DataAssociation>& data_associations_;
//        const std::vector<std::shared_ptr<std::vector<Pointd> > >& features_;
//        const std::vector<int> block_sizes_;
//
//
//    public:
//        LidarAllCostFunction(
//                const State& state
//                , const std::vector<DataAssociation>& data_associations
//                , const std::vector<std::shared_ptr<std::vector<Pointd> > >& features)
//                : state_(state)
//                , data_associations_(data_associations)
//                , features_(features)
//                , block_sizes_({3,3,3,3,1,7})
//        {
//            set_num_residuals(data_associations_.size());
//
//            mutable_parameter_block_sizes()->push_back(3);
//            mutable_parameter_block_sizes()->push_back(3);
//            mutable_parameter_block_sizes()->push_back(3);
//            mutable_parameter_block_sizes()->push_back(3);
//            mutable_parameter_block_sizes()->push_back(1);
//            mutable_parameter_block_sizes()->push_back(7);
//
//            for(size_t i = 0; i < features_.size(); ++i)
//            {
//                feature_times_.push_back(std::vector<double>(features_[i]->size()));
//                for(size_t j = 0; j < features_[i]->size(); ++j)
//                {
//                    feature_times_[i][j] = features_[i]->at(j).t;
//                }
//            }
//        }
//
//        bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
//        {
//            std::cout << "=====Cost function time (jacobians = " << (jacobians != NULL) << ") with " << features_.size() <<" scans" << std::endl;
//            StopWatch sw;
//            sw.start();
//
//            Eigen::Map<const Vec3> arg_0(parameters[0]);
//            Eigen::Map<const Vec3> arg_1(parameters[1]);
//            Eigen::Map<const Vec3> arg_2(parameters[2]);
//            Eigen::Map<const Vec3> arg_3(parameters[3]);
//            double time_offset = parameters[4][0];
//            Eigen::Map<const Vec4> quat_I_L(parameters[5]);
//            Eigen::Map<const Vec3> pos_I_L(parameters[5]+4);
//
//            Eigen::Map<VecX> res(residuals, data_associations_.size());
//
//            std::vector<std::vector<std::pair<Vec3, Vec3> > > poses;
//            std::vector<std::vector<std::vector<std::pair<MatX, MatX> > > > pose_jacobians;
//            if(jacobians != NULL)
//            {
//                for(size_t i = 0; i < features_.size(); ++i)
//                {
//                    StopWatch sw2;
//                    sw2.start();
//                    auto [temp_poses, temp_pose_jacobians] = state_.queryWthJacobian(feature_times_[i], arg_0, arg_1, arg_2, arg_3, time_offset);
//                    sw2.stop();
//                    sw2.print("Queried state for one scan");
//                    sw2.reset();
//                    sw2.start();
//                    poses.push_back(temp_poses);
//                    pose_jacobians.push_back(temp_pose_jacobians);
//                    sw2.stop();
//                    sw2.print("Copied state for one scan");
//                }
//            }
//            else
//            {
//                for(size_t i = 0; i < features_.size(); ++i)
//                {
//                    auto temp_poses = state_.query(feature_times_[i], arg_0, arg_1, arg_2, arg_3, time_offset);
//                    poses.push_back(temp_poses);
//                }
//            }
//
//            sw.stop();
//            sw.print("Queried state for all features");
//
//            sw.reset();
//            sw.start();
//
//            #pragma omp parallel for
//            for(size_t i = 0; i < data_associations_.size(); ++i)
//            {
//                Vec3 feature_L = features_[data_associations_[i].pc_id]->at(data_associations_[i].feature_id).vec3();
//                Vec3 feature_I;
//                ceres::UnitQuaternionRotatePoint(quat_I_L.data(), feature_L.data(), feature_I.data());
//                feature_I += pos_I_L;
//                Vec3& feature_rot = poses[data_associations_[i].pc_id][data_associations_[i].feature_id].second;
//                Vec3& feature_pos = poses[data_associations_[i].pc_id][data_associations_[i].feature_id].first;
//
//                Vec3 feature_W_rot;
//                ceres::AngleAxisRotatePoint(feature_rot.data(), feature_I.data(), feature_W_rot.data());
//                Vec3 feature_W = feature_W_rot + feature_pos;
//
//                std::vector<Vec3> target_rots(data_associations_[i].target_ids.size());
//                std::vector<Vec3> targets_W_rot(data_associations_[i].target_ids.size());
//                std::vector<Vec3> targets_W(data_associations_[i].target_ids.size());
//                for(size_t j = 0; j < data_associations_[i].target_ids.size(); ++j)
//                {
//                    Vec3 target_L = features_[data_associations_[i].target_ids[j].first]->at(data_associations_[i].target_ids[j].second).vec3();
//                    Vec3 target_I;
//                    ceres::UnitQuaternionRotatePoint(quat_I_L.data(), target_L.data(), target_I.data());
//                    target_I += pos_I_L;
//                    Vec3& target_rot = poses[data_associations_[i].target_ids[j].first][data_associations_[i].target_ids[j].second].second;
//                    Vec3& target_pos = poses[data_associations_[i].target_ids[j].first][data_associations_[i].target_ids[j].second].first;
//
//                    Vec3 target_W;
//                    ceres::AngleAxisRotatePoint(target_rot.data(), target_I.data(), target_W.data());
//                    target_rots[j] = target_rot;
//                    targets_W_rot[j] = target_W;
//                    targets_W[j] = target_W + target_pos;
//                }
//
//
//                res[i] = data_associations_[i].computeResidual(feature_W, targets_W);
//
//                if(jacobians != NULL)
//                {
//                    RowX d_res_d_pts = data_associations_[i].computeJacobian(feature_W, targets_W);
//                    Mat3 d_feature_d_rot = -ugpm::toSkewSymMat(feature_W_rot)*ugpm::jacobianRighthandSO3(-feature_rot);
//                    std::vector<Mat3> d_target_d_rot(data_associations_[i].target_ids.size());
//                    for(size_t j = 0; j < data_associations_[i].target_ids.size(); ++j)
//                    {
//                        Vec3& target_rot = poses[data_associations_[i].target_ids[j].first][data_associations_[i].target_ids[j].second].second;
//                        d_target_d_rot[j] = -ugpm::toSkewSymMat(targets_W_rot[j])*ugpm::jacobianRighthandSO3(-target_rot);
//                    }
//
//
//                    for(int j = 0; j < 5; ++j)
//                    {
//                        if(jacobians[j] != NULL)
//                        {
//                            int block_size = block_sizes_[j];
//                            Eigen::Map<Eigen::Matrix<double, 1,Eigen::Dynamic> > j_s(&(jacobians[j][i*block_size]),1,block_size);
//                            j_s.setZero();
//                            if(pose_jacobians[data_associations_[i].pc_id][data_associations_[i].feature_id][j].first.size() > 0)
//                            {
//                                j_s += d_res_d_pts.segment<3>(0) * pose_jacobians[data_associations_[i].pc_id][data_associations_[i].feature_id][j].first;
//                            }
//                            if(pose_jacobians[data_associations_[i].pc_id][data_associations_[i].feature_id][j].second.size() > 0)
//                            {
//                                j_s += d_res_d_pts.segment<3>(0) * d_feature_d_rot * pose_jacobians[data_associations_[i].pc_id][data_associations_[i].feature_id][j].second;
//                            }
//                            for(size_t k = 0; k < data_associations_[i].target_ids.size(); ++k)
//                            {
//                                if(pose_jacobians[data_associations_[i].target_ids[k].first][data_associations_[i].target_ids[k].second][j].first.size() > 0)
//                                {
//                                    j_s += d_res_d_pts.segment<3>(3*(k+1)) * pose_jacobians[data_associations_[i].target_ids[k].first][data_associations_[i].target_ids[k].second][j].first;
//                                }
//                                if(pose_jacobians[data_associations_[i].target_ids[k].first][data_associations_[i].target_ids[k].second][j].second.size() > 0)
//                                {
//                                    j_s += d_res_d_pts.segment<3>(3*(k+1)) * d_target_d_rot[k] * pose_jacobians[data_associations_[i].target_ids[k].first][data_associations_[i].target_ids[k].second][j].second;
//                                }
//                            }
//                        }
//                    }
//
//                    if(jacobians[5] != NULL)
//                    {
//                        Mat3 R_W_I = ugpm::expMap(feature_rot);
//                        std::vector<Mat3> target_R_W_I(data_associations_[i].target_ids.size());
//                        for(size_t k = 0; k < data_associations_[i].target_ids.size(); ++k)
//                        {
//                            target_R_W_I[k] = ugpm::expMap(target_rots[k]);
//                        }
//
//                        Eigen::Map<Eigen::Matrix<double, 1, 7> > j_s(&(jacobians[5][i*7]));
//                        j_s.setOnes();
//                        MatX d_pts_d_s(3+(3*data_associations_[i].target_ids.size()), 7);
//                        d_pts_d_s.block<3,4>(0,0) = R_W_I*jacobianQuatRotation(quat_I_L, feature_L);
//                        d_pts_d_s.block<3,3>(0,4) = R_W_I;
//                        for(size_t k = 0; k < data_associations_[i].target_ids.size(); ++k)
//                        {
//                            d_pts_d_s.block<3,4>(3*(k+1),0) = target_R_W_I[k]*jacobianQuatRotation(quat_I_L, features_[data_associations_[i].target_ids[k].first]->at(data_associations_[i].target_ids[k].second).vec3());
//                            d_pts_d_s.block<3,3>(3*(k+1),4) = target_R_W_I[k];
//                        }
//
//                        j_s = d_res_d_pts * d_pts_d_s; 
//                    }
//                }
//
//            }
//
//            sw.stop();
//            sw.print("Computed residuals and jacobians");
//
//            return true;
//        }
//};





//class LidarCostFunction: public ceres::CostFunction
//{
//    private:
//        const int nb_state_;
//        const DataAssociation data_association_;
//        const std::vector<std::shared_ptr<std::vector<Pointd> > >& features_;
//        const std::vector<std::vector<MatX> >& ks_int_K_inv_;
//        const std::vector<std::vector<MatX> >& ks_int2_K_inv_;
//        const double weight_;
//        double feature_dt_;
//        std::vector<double> target_dt_;
//
//    public:
//        LidarCostFunction(const DataAssociation& data_association, const std::vector<std::shared_ptr<std::vector<Pointd> > >& features, const std::vector<std::vector<MatX> >& ks_int_K_inv, const std::vector<std::vector<MatX> >& ks_int2_K_inv, const double anchor_t, const double point_std = 0.02)
//                : data_association_(data_association)
//                , features_(features)
//                , ks_int_K_inv_(ks_int_K_inv)
//                , ks_int2_K_inv_(ks_int2_K_inv)
//                , nb_state_(ks_int_K_inv[0][0].cols())
//                , weight_(1.0/point_std)
//        {
//            feature_dt_ = features_[data_association_.pc_id]->at(data_association_.feature_id).t - anchor_t;
//            target_dt_.resize(data_association_.target_ids.size());
//            for(int i = 0; i < data_association_.target_ids.size(); ++i)
//            {
//                target_dt_[i] = features_[data_association_.target_ids[i].first]->at(data_association_.target_ids[i].second).t - anchor_t;
//            }
//
//
//            set_num_residuals(data_association_.residualSize());
//            mutable_parameter_block_sizes()->push_back(nb_state_);
//            mutable_parameter_block_sizes()->push_back(nb_state_);
//            mutable_parameter_block_sizes()->push_back(nb_state_);
//            mutable_parameter_block_sizes()->push_back(nb_state_);
//            mutable_parameter_block_sizes()->push_back(nb_state_);
//            mutable_parameter_block_sizes()->push_back(nb_state_);
//            mutable_parameter_block_sizes()->push_back(3);
//            mutable_parameter_block_sizes()->push_back(7);
//        }
//
//        bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
//        {
//            // Read the parameters
//            Eigen::Map<const VecX> state_acc_0(parameters[0], nb_state_);
//            Eigen::Map<const VecX> state_acc_1(parameters[1], nb_state_);
//            Eigen::Map<const VecX> state_acc_2(parameters[2], nb_state_);
//            Eigen::Map<const VecX> state_angvel_0(parameters[3], nb_state_);
//            Eigen::Map<const VecX> state_angvel_1(parameters[4], nb_state_);
//            Eigen::Map<const VecX> state_angvel_2(parameters[5], nb_state_);
//            Eigen::Map<const Vec3> vel(parameters[6]);
//            Eigen::Map<const Vec4> quat_I_L(parameters[7]);
//            Eigen::Map<const Vec3> pos_I_L(parameters[7]+4);
//
//            // Get the rotation and translation from the state for the feature and the targets
//            const MatX& ks_feature_int_K_inv = ks_int_K_inv_[data_association_.pc_id][data_association_.feature_id];
//            const MatX& ks_feature_int2_K_inv = ks_int2_K_inv_[data_association_.pc_id][data_association_.feature_id];
//            Vec3 feature_rot;
//            feature_rot[0] = (ks_feature_int_K_inv*state_angvel_0)(0,0);
//            feature_rot[1] = (ks_feature_int_K_inv*state_angvel_1)(0,0);
//            feature_rot[2] = (ks_feature_int_K_inv*state_angvel_2)(0,0);
//            Vec3 feature_pos;
//            feature_pos[0] = (ks_feature_int2_K_inv*state_acc_0)(0,0);
//            feature_pos[1] = (ks_feature_int2_K_inv*state_acc_1)(0,0);
//            feature_pos[2] = (ks_feature_int2_K_inv*state_acc_2)(0,0);
//            feature_pos += vel*feature_dt_;
//
//            std::vector<Vec3> target_rots(data_association_.target_ids.size());
//            std::vector<Vec3> target_pos(data_association_.target_ids.size());
//
//            for(int i = 0; i < data_association_.target_ids.size(); ++i)
//            {
//                const MatX& ks_target_int_K_inv = ks_int_K_inv_[data_association_.target_ids[i].first][data_association_.target_ids[i].second];
//                const MatX& ks_target_int2_K_inv = ks_int2_K_inv_[data_association_.target_ids[i].first][data_association_.target_ids[i].second];
//                target_rots[i][0] = (ks_target_int_K_inv*state_angvel_0)(0,0);
//                target_rots[i][1] = (ks_target_int_K_inv*state_angvel_1)(0,0);
//                target_rots[i][2] = (ks_target_int_K_inv*state_angvel_2)(0,0);
//                target_pos[i][0] = (ks_target_int2_K_inv*state_acc_0)(0,0);
//                target_pos[i][1] = (ks_target_int2_K_inv*state_acc_1)(0,0);
//                target_pos[i][2] = (ks_target_int2_K_inv*state_acc_2)(0,0);
//                target_pos[i] += vel*target_dt_[i];
//            }
//
//            // Transform the feature and the targets
//            Vec3 feature_L = features_[data_association_.pc_id]->at(data_association_.feature_id).vec3();
//            Vec3 feature_I;
//            ceres::UnitQuaternionRotatePoint(quat_I_L.data(), feature_L.data(), feature_I.data());
//            feature_I += pos_I_L;
//            Vec3 feature_W_rot;
//            ceres::AngleAxisRotatePoint(feature_rot.data(), feature_I.data(), feature_W_rot.data());
//            Vec3 feature_W = feature_W_rot + feature_pos;
//
//            std::vector<Vec3> targets_W_rot(data_association_.target_ids.size());
//            std::vector<Vec3> targets_W(data_association_.target_ids.size());
//            for(int i = 0; i < data_association_.target_ids.size(); ++i)
//            {
//                Vec3 target_L = features_[data_association_.target_ids[i].first]->at(data_association_.target_ids[i].second).vec3();
//                Vec3 target_I;
//                ceres::UnitQuaternionRotatePoint(quat_I_L.data(), target_L.data(), target_I.data());
//                target_I += pos_I_L;
//                Vec3 target_W;
//                ceres::AngleAxisRotatePoint(target_rots[i].data(), target_I.data(), target_W.data());
//                targets_W_rot[i] = target_W;
//                targets_W[i] = target_W + target_pos[i];
//            }
//
//            // Compute the residual
//            Eigen::Map<VecX> res(residuals, data_association_.residualSize());
//            res = weight_ * data_association_.computeResidual(feature_W, targets_W);
//
//
//            // Compute the jacobian
//            if(jacobians != NULL)
//            {
//                MatX d_res_d_pts = weight_*data_association_.computeJacobian(feature_W, targets_W);
//                Mat3 R_W_I = ugpm::expMap(feature_rot);
//                std::vector<Mat3> target_R_W_I(data_association_.target_ids.size());
//                for(int i = 0; i < data_association_.target_ids.size(); ++i)
//                {
//                    target_R_W_I[i] = ugpm::expMap(target_rots[i]);
//                }
//
//                
//                if((jacobians[0] != NULL)||(jacobians[1] != NULL)||(jacobians[2] != NULL))
//                {
//                    for(int axis=0; axis<3; ++axis)
//                    {
//                        if(jacobians[axis] != 0)
//                        {
//                            Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> > j_s(&(jacobians[axis][0]),data_association_.residualSize(),nb_state_);
//                            j_s = d_res_d_pts.col(axis) * ks_feature_int2_K_inv;
//                            for(int i = 0; i < data_association_.target_ids.size(); ++i)
//                            {
//                                const MatX& ks_target_int2_K_inv = ks_int2_K_inv_[data_association_.target_ids[i].first][data_association_.target_ids[i].second];
//                                j_s += d_res_d_pts.col(3*(i+1)+axis) * ks_target_int2_K_inv;
//                            }
//                        }
//                    }
//                }
//
//                if((jacobians[3] != NULL)||(jacobians[4] != NULL)||(jacobians[5] != NULL))
//                {
//                    std::vector<MatX> d_res_d_r(1+data_association_.target_ids.size());
//                    d_res_d_r[0] = d_res_d_pts.block(0,0, data_association_.residualSize(), 3) * (-ugpm::toSkewSymMat(feature_W_rot)*ugpm::jacobianRighthandSO3(-feature_rot));
//                    for (int i = 0; i < data_association_.target_ids.size(); ++i)
//                    {
//                        d_res_d_r[i+1] = d_res_d_pts.block(0,3*(i+1), data_association_.residualSize(), 3) * (-ugpm::toSkewSymMat(targets_W_rot[i])*ugpm::jacobianRighthandSO3(-target_rots[i]));
//                    }
//                    for(int axis=0; axis<3; ++axis)
//                    {
//                        if(jacobians[axis+3] != 0)
//                        {
//                            Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> > j_s(&(jacobians[axis+3][0]),data_association_.residualSize(),nb_state_);
//                            j_s = d_res_d_r[0].col(axis) * ks_feature_int_K_inv;
//                            for(int i = 0; i < data_association_.target_ids.size(); ++i)
//                            {
//                                const MatX& ks_target_int_K_inv = ks_int_K_inv_[data_association_.target_ids[i].first][data_association_.target_ids[i].second];
//                                j_s += d_res_d_r[i+1].col(axis) * ks_target_int_K_inv;
//                            }
//                        }
//                    }
//                }
//
//                if(jacobians[6] != NULL)
//                {
//                    Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> > j_s(&(jacobians[6][0]),data_association_.residualSize(),3);
//                    j_s = d_res_d_pts.block(0,0, data_association_.residualSize(), 3)*feature_dt_;
//                    for(int i = 0; i < data_association_.target_ids.size(); ++i)
//                    {
//                        j_s += d_res_d_pts.block(0,3*(i+1), data_association_.residualSize(), 3)*target_dt_[i];
//                    }
//                }
//
//                if(jacobians[7] != NULL)
//                {
//                    Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> > j_s(&(jacobians[7][0]),data_association_.residualSize(),7);
//                    
//                    MatX d_pts_d_s(3+(3*data_association_.target_ids.size()), 7);
//                    d_pts_d_s.block<3,4>(0,0) = R_W_I*jacobianQuatRotation(quat_I_L, feature_L);
//                    d_pts_d_s.block<3,3>(0,4) = R_W_I;
//                    for(int i = 0; i < data_association_.target_ids.size(); ++i)
//                    {
//                        d_pts_d_s.block<3,4>(3*(i+1),0) = target_R_W_I[i]*jacobianQuatRotation(quat_I_L, features_[data_association_.target_ids[i].first]->at(data_association_.target_ids[i].second).vec3());
//                        d_pts_d_s.block<3,3>(3*(i+1),4) = target_R_W_I[i];
//                    }
//
//                    j_s = d_res_d_pts * d_pts_d_s;
//                }
//            }
//
//            return true;
//        }
//};

