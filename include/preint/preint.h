#ifndef UGPM_2_H
#define UGPM_2_H


#include "types.h"
#include "cost_functions.h"
#include "math.h"
#include <ceres/ceres.h>

#include <ctime>
#include <chrono>
#include <mutex>
#include <thread>

namespace ugpm
{
    enum QueryType { kVecVec, kVec, kSingle };

    const int kOverlap = 8;

    // Class to compute preintegrated measurements and potentially store the pre-computed meausrements
    class ImuPreintegration
    {


        public:
            // Constructor given IMU data and inference timestamps
            ImuPreintegration(const ImuData& imu_data,
                    const double start_t,
                    const std::vector<std::vector<double> >& infer_t,
                    const PreintOption opt,
                    const PreintPrior prior,
                    const bool rot_only = false,
                    const int overlap = kOverlap);

            // Constructor overloading given only a vector of timestamps
            ImuPreintegration(const ImuData& imu_data,
                    const double start_t,
                    const std::vector<double>& infer_t,
                    const PreintOption opt,
                    const PreintPrior prior,
                    const bool rot_only = false,
                    const int overlap = kOverlap);

            // Constructor overloading given only a one timestamp
            ImuPreintegration(const ImuData& imu_data,
                    const double start_t,
                    const double infer_t,
                    const PreintOption opt,
                    const PreintPrior prior,
                    const bool rot_only = false,
                    const int overlap = kOverlap);

            // Get the preintegrated measurement as per indexed in the given inference times of the constructor
            PreintMeas get(const int index_1, const int index_2, double acc_bias_std = 0.3, double gyr_bias_std = 0.03);

            // Overload of the get function when vector in the constructor
            PreintMeas get(const int index_1, double acc_bias_std = 0.3, double gyr_bias_std = 0.03);

            // Overload of the get function when single timestamp in the constructor
            PreintMeas get(double acc_bias_std = 0.3, double gyr_bias_std = 0.03);

            // Accessor for prior
            PreintPrior getPrior() { return prior_;}

        private:
            // Store a copy of the imu_data used
            ImuData imu_data_;

            double start_t_;
            size_t start_index_;
            PreintOption opt_;
            PreintPrior prior_;


            // Store the pre-computed preintegrated measurements if contructor with time-stamps
            std::vector<std::vector<PreintMeas> > preint_;

            // Store the type of query
            QueryType query_type_ = kVecVec;

    };


    class StopWatch
    {   
        public:
            StopWatch()
            {
                duration_ = std::chrono::high_resolution_clock::duration::zero();
            }

            //Start the counting (also used to get time while running);
            double start()
            {
                double output = 0.0;
                auto temp = std::chrono::high_resolution_clock::now();
                if(!stopped_)
                {
                    duration_ += (temp - last_);
                    output = std::chrono::duration_cast<std::chrono::microseconds>(duration_).count();
                }
                last_ = temp;
                stopped_ = false;
                return output/1000.0;
            }

            // Since the first "start after the last reset"
            double getTotal()
            {
                auto temp = std::chrono::high_resolution_clock::now();
                return std::chrono::duration_cast<std::chrono::microseconds>(duration_ + (temp - last_)).count()/1000.0;
            }

            // Since the 
            double getLast()
            {
                auto temp = std::chrono::high_resolution_clock::now();
                return std::chrono::duration_cast<std::chrono::microseconds>(temp - last_).count()/1000.0;

            }

            double stop()
            {
                if(!stopped_)
                {
                    auto temp = std::chrono::high_resolution_clock::now();
                    stopped_ = true;
                    duration_ += (temp - last_);
                    return std::chrono::duration_cast<std::chrono::microseconds>(duration_).count()/1000.0;
                }
                else
                {
                    std::cout << "WARNING: Stopping a StopWatch that is not running, returning negative time" << std::endl;
                    return -1;
                }
            }
            void reset()
            {
                stopped_ = true;
                duration_ = std::chrono::high_resolution_clock::duration::zero();
            }


            void print()
            {
                print("Time elapsed:");
            }

            void print(std::string str)
            {
                std::cout << str << " " << std::chrono::duration_cast<std::chrono::microseconds>(duration_).count()/1000.0 << " ms" << std::endl;
            }

        private:
            double counter_ = 0;
            bool stopped_ = true;
            std::chrono::high_resolution_clock::duration duration_;
            std::chrono::high_resolution_clock::time_point last_;
    };








    // Iterative integrator of imu data (still need batch of data but the algorithm is mostly iterative)
    class IterativeIntegrator{

        public:
            IterativeIntegrator(
                const ImuData& imu_data,
                const double start_time,
                const PreintPrior bias_prior,
                const std::vector<std::vector<double> >& time,
                const double min_freq = 500,
                const bool bare = false,
                const bool rot_only = false)
            {
                start_t_ = start_time;
                bare_ = bare;

                size_t nb_infer_vec = time.size();
                
                // Fill the private structures and other variables
                nb_gyr_ = imu_data.gyr.size();
                nb_acc_ = imu_data.acc.size();
                gyr_var_ = imu_data.gyr_var;
                acc_var_ = imu_data.acc_var;
                gyr_data_.resize(3, nb_gyr_);
                acc_data_.resize(3, nb_acc_);
                gyr_time_.resize(nb_gyr_);
                acc_time_.resize(nb_acc_);
                for(int i = 0; i < nb_gyr_; ++i)
                {
                    gyr_data_(0,i) = imu_data.gyr[i].data[0] - bias_prior.gyr_bias[0];
                    gyr_data_(1,i) = imu_data.gyr[i].data[1] - bias_prior.gyr_bias[1];
                    gyr_data_(2,i) = imu_data.gyr[i].data[2] - bias_prior.gyr_bias[2];
                    gyr_time_(i) = imu_data.gyr[i].t;
                }
                for(int i = 0; i < nb_acc_; ++i)
                {
                    acc_data_(0,i) = imu_data.acc[i].data[0] - bias_prior.acc_bias[0];
                    acc_data_(1,i) = imu_data.acc[i].data[1] - bias_prior.acc_bias[1];
                    acc_data_(2,i) = imu_data.acc[i].data[2] - bias_prior.acc_bias[2];
                    acc_time_(i) = imu_data.acc[i].t;
                }



                // Create a vector of timestamps of interest includung the starting timestamp (and its shifted value) and the timestamps of the accerlerometer data
                std::vector<std::vector<double> > infer_t;
                for(const auto& v: time) infer_t.push_back(v);
                std::vector<double> temp_infer_t1;
                temp_infer_t1.push_back(start_t_);
                temp_infer_t1.push_back(start_t_+kNumDtJacobianDelta);
                infer_t.push_back(temp_infer_t1);
                std::vector<double> temp_infer_t2;
                for(int i = 0; i < nb_acc_; ++i) temp_infer_t2.push_back(acc_time_(i));
                infer_t.push_back(temp_infer_t2);

                // Sort the timestamps
                SortIndexTracker2<double> t(infer_t);

                // Check if the data has high enough frequency, otherwise add fake timestamps
                if(t.getSmallestGap() > (1.0/min_freq))
                {
                    std::vector<double> fake_time;
                    int nb_fake = std::floor((t.back() - t.get(0)) * min_freq);
                    double offset = t.get(0);
                    double quantum = (t.back() - t.get(0)) / ((double) nb_fake);
                    for(int i = 0; i < nb_fake; ++i) fake_time.push_back(offset + (i*quantum));
                    infer_t.push_back(fake_time);
                    t = SortIndexTracker2<double>(infer_t);
                }

                start_index_ = t.getIndex(nb_infer_vec, 0);

                // Vector of flag of timestamp interest to later prevent unnecessary computations
                std::vector<bool> interest_t;
                {
                    std::vector<std::vector<bool> > temp;
                    for(size_t i = 0; i < nb_infer_vec; ++i) temp.push_back(std::vector<bool>(infer_t[i].size(), true));
                    temp.push_back(std::vector<bool>(infer_t[nb_infer_vec].size(), true));
                    temp.push_back(std::vector<bool>(infer_t[nb_infer_vec+1].size(), true));
                    if( (nb_infer_vec+2) < infer_t.size()) temp.push_back(std::vector<bool>(infer_t[nb_infer_vec+2].size(), false));
                    interest_t = t.applySort(temp);
                }


                // Compute the rotational part of the preintegration 
                std::vector<PreintMeas> preint = rotPreint(t, interest_t);

                // Demux the preintegrated mesurements
                for(size_t i = 0; i < nb_infer_vec; ++i)
                {
                    preint_.push_back(t.getVector(preint, i));

                }

                if(rot_only)
                {
                    return;
                }

                Mat3 delta_R_dt_start = t.get(nb_infer_vec, 1, preint).delta_R;


                // Reproject the accelerometer data (and apply the prior)
                std::vector<PreintMeas> acc_time_preint = t.getVector(preint, nb_infer_vec+1);
                if(!bare_)
                {
                    std::vector<Eigen::MatrixXd> d_acc_d_bf;
                    std::vector<Eigen::MatrixXd> d_acc_d_bw;
                    std::vector<Eigen::VectorXd> d_acc_d_dt;
                    acc_data_ = reprojectAccData(acc_time_preint, acc_data_, delta_R_dt_start, d_acc_d_bf, d_acc_d_bw, d_acc_d_dt);
                    while(infer_t.size() != nb_infer_vec)
                    {
                        infer_t.pop_back();
                    }
                    velPosPreintLPM(infer_t, d_acc_d_bf, d_acc_d_bw, d_acc_d_dt, preint_);
                }
                else
                {
                    acc_data_ = reprojectAccData(acc_time_preint, acc_data_);
                    while(infer_t.size() != nb_infer_vec)
                    {
                        infer_t.pop_back();
                    }
                    SortIndexTracker2<double> temp_t(infer_t);
                    velPosPreintLPMPartial(temp_t, preint_);

                }
            }

            
            // Get the preintegrated measurement as per indexed in the given inference times of the constructor
            PreintMeas get(const int index_1, const int index_2)
            {
                return preint_[index_1][index_2];
            }

        private:
            double start_t_;
            std::vector<std::vector<PreintMeas> > preint_;
            bool bare_;

            size_t start_index_;

            int nb_gyr_;
            int nb_acc_;
            double gyr_var_;
            double acc_var_;
            Eigen::MatrixXd gyr_data_;
            Eigen::MatrixXd acc_data_;
            Eigen::VectorXd gyr_time_;
            Eigen::VectorXd acc_time_;

            std::vector<PreintMeas> rotPreint(const SortIndexTracker2<double>& t,
                        const std::vector<bool>& interest_t)
            {
                std::vector<PreintMeas> output(t.size());

                MatX inter_w(3,t.size());
                MatX inter_var(3,t.size());
                MatX inter_w_shifted;
                VecX gyr_time_shifted;
                if(!bare_)
                {
                    inter_w_shifted.resize(3,t.size());
                    gyr_time_shifted = (gyr_time_.array() - kNumDtJacobianDelta).matrix();
                }

                for(int i = 0; i < 3; ++i)
                {
                    auto [val, var] = linearInterpolation(gyr_data_.row(i).transpose(), gyr_time_, gyr_var_, t);
                    inter_w.row(i) = val.transpose();
                    inter_var.row(i) = var.transpose();
                    if(!bare_)
                    {
                        VecX val_shifted = linearInterpolation(gyr_data_.row(i).transpose(), gyr_time_shifted, t);
                        inter_w_shifted.row(i) = val_shifted.transpose();
                    }
                }

                if(!bare_)
                {
                    rotIterativeIntegration(inter_w, inter_var, t, output);

                    // Compute the numerical Jacobians for post integration 
                    std::vector<Mat3, Eigen::aligned_allocator<Mat3> > d_R_dt(output.size());
                    std::vector<std::vector<Mat3, Eigen::aligned_allocator<Mat3>> > d_R_d_bw(3, std::vector<Mat3, Eigen::aligned_allocator<Mat3> >(output.size()));
                    
                    rotIterativeIntegration(inter_w_shifted, t, d_R_dt);
                    for(size_t j = 0; j < t.size(); ++j)
                    {
                        if(interest_t[j])
                        {
                            output[j].d_delta_R_d_t = logMap(output[j].delta_R.transpose()*(d_R_dt[j]))/kNumDtJacobianDelta;
                        }
                    }

                    for(int i = 0; i < 3; ++i)
                    {
                        MatX temp_data = inter_w;
                        Vec3 offset = Vec3::Zero();
                        offset(i) = kNumGyrBiasJacobianDelta;
                        temp_data.colwise() += offset;
                        rotIterativeIntegration(temp_data, t, d_R_d_bw[i]);
                        for(size_t j = 0; j < t.size(); ++j)
                        {
                            if(interest_t[j])
                            {
                                output[j].d_delta_R_d_bw.col(i) = logMap(output[j].delta_R.transpose()*(d_R_d_bw[i][j]))/kNumGyrBiasJacobianDelta;
                            }
                        }
                    }
                }
                else
                {
                    std::vector<Mat3, Eigen::aligned_allocator<Mat3> > temp_R(t.size());
                    rotIterativeIntegration(inter_w, t, temp_R);
                    for(size_t i = 0; i < t.size(); ++i)
                    {
                        output[i].delta_R = temp_R[i];
                    }
                }
                return output;
            }

            template<typename T>
            T minCovDiag(const T& cov, const double min_val=1e-6)
            {
                T output = cov;
                for(int i = 0; i < cov.rows(); ++i)
                {
                    if(output(i,i) < min_val)
                    {
                        output(i,i) = min_val;
                    }
                }
                return output;
            }

            void rotIterativeIntegration(const Eigen::MatrixXd& w, const Eigen::MatrixXd& var, const SortIndexTracker2<double>& t, std::vector<PreintMeas>& output)
            {

                Mat3 rot_mat = Mat3::Identity();
                Mat9 cov = Mat9::Zero();

                output[0].delta_R = rot_mat;
                output[0].cov = minCovDiag(cov);

                output[0].dt = t.get(0) - start_t_;
                output[0].dt_sq_half = 0.5*(output[0].dt)*(output[0].dt);

                
                for(size_t i = 0; i < (t.size() - 1); ++i)
                {
                    // Prepare some variables for convenience sake
                    double dt = t.get(i+1)-t.get(i);
                    Vec3 gyr_dt = w.col(i) * dt;
                    
                    double gyr_norm = gyr_dt.norm();

                    Mat3 e_R = Mat3::Identity();
                    Mat3 j_r = Mat3::Identity();

                    // If the angular velocity norm is non-null
                    if(gyr_norm > 0.0000000001){

                        // Turn the angular velocity into skew-simmetric matrix
                        Mat3 gyr_skew_mat;
                        gyr_skew_mat <<     0, -gyr_dt[2],  gyr_dt[1],
                                    gyr_dt[2],         0, -gyr_dt[0],
                                    -gyr_dt[1],  gyr_dt[0],         0;

                        
                        double s_gyr_norm = std::sin(gyr_norm);
                        double gyr_norm_sq = gyr_norm * gyr_norm;
                        double scalar_2 = (1 - std::cos(gyr_norm)) / gyr_norm_sq;
                        Mat3 skew_mat_sq = gyr_skew_mat * gyr_skew_mat;

                        e_R = e_R + ( (s_gyr_norm / gyr_norm ) * gyr_skew_mat )
                                    + ( scalar_2 * skew_mat_sq);

                        j_r = j_r - (scalar_2 * gyr_skew_mat)
                                    + ( ( (gyr_norm - s_gyr_norm)/ (gyr_norm_sq * gyr_norm) ) * skew_mat_sq);

                    }


                    if( (i+1) > start_index_ ){

                        Mat3 A = e_R.transpose();
                        Mat3 B = (j_r*dt);
                        Mat3 imu_cov;
                        imu_cov <<  var(0,i), 0.0, 0.0,
                                    0.0, var(1,i), 0.0,
                                    0.0, 0.0, var(2,i);
                        cov.block<3,3>(0,0) = A*cov.block<3,3>(0,0)*A.transpose()
                            + B*imu_cov*B.transpose();
                    }

                    rot_mat = rot_mat * e_R;
                    output[i+1].delta_R = rot_mat;
                    output[i+1].cov = minCovDiag(cov);
                    output[i+1].dt = t.get(i+1) - start_t_;
                    output[i+1].dt_sq_half = output[i+1].dt*output[i+1].dt*0.5;

                    

                    // Reproject the preintegrated measurements from in the starting frame
                    if( (i+1) == start_index_)
                    {
                        for(size_t j = 0; j < (i+1); ++j)
                        {                        
                            output[j].delta_R = rot_mat.transpose()*output[j].delta_R;
                        }
                        rot_mat = Mat3::Identity();
                        output[start_index_].delta_R = rot_mat;
                    }
                }
            }

            void rotIterativeIntegration(const Eigen::MatrixXd& w, const SortIndexTracker2<double>& t, std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >& output)
            {
                Mat3 rot_mat = Mat3::Identity();

                output[0] = rot_mat;
                
                for(size_t i = 0; i < (t.size() - 1); ++i)
                {
                    // Prepare some variables for convenience sake
                    double dt = t.get(i+1)-t.get(i);
                    
                    Vec3 gyr_dt = w.col(i) * dt;

                    Mat3 e_R = expMap(gyr_dt);

                    rot_mat = rot_mat * e_R;
                    output[i+1] = rot_mat;

                    // Reproject the preintegrated measurements from in the starting frame
                    if( (i+1) == start_index_)
                    {
                        for(size_t j = 0; j < (i+1); ++j)
                        {                        
                            output[j] = rot_mat.transpose()*output[j];
                        }
                        rot_mat = Mat3::Identity();
                        output[start_index_] = rot_mat;
                    }
                }
            }



            // Compute the preintegration for the velocity and position part
            void velPosPreintLPM(
                        std::vector<std::vector<double> >& t,
                        const std::vector<MatX>& d_acc_d_bf,
                        const std::vector<MatX>& d_acc_d_bw,
                        const std::vector<VecX>& d_acc_d_dt,
                        std::vector<std::vector<PreintMeas> >& preint)
            {

                SortIndexTracker2<double> time(t);

                // Compute with the time shift (could be optimised somehow I guess)
                MatX save_acc_data = acc_data_;
                VecX save_acc_time = acc_time_;
                for(int i = 0; i < nb_acc_; ++i)
                {
                    Vec3 temp_d_acc_d_dt;
                    temp_d_acc_d_dt <<  d_acc_d_dt[0].coeff(i), 
                                        d_acc_d_dt[1].coeff(i), 
                                        d_acc_d_dt[2].coeff(i);
                    acc_data_.col(i) += kNumDtJacobianDelta*temp_d_acc_d_dt;
                    acc_time_[i] -= kNumDtJacobianDelta;
                }
                velPosPreintLPMPartial(time, preint);

                acc_data_ = save_acc_data;
                acc_time_ = save_acc_time;


                int data_ptr = 0;

                // For each of the query points of the timeline
                size_t start_index = 0;
                while(time.get(start_index) < start_t_){
                    start_index++;
                    if(start_index == time.size()){
                        throw "FullLPM: the start_time is not in the query domain";
                    }
                }
                while(acc_time_[data_ptr+1] < start_t_){
                    data_ptr++;
                    if(data_ptr == (nb_acc_-1)){
                        throw "FullLPM: the start_time is not in the data domain";
                    }
                }



                for(int axis = 0; axis < 3; ++axis)
                {
                    int ptr = data_ptr;
                    double alpha = (acc_data_(axis,ptr+1) - acc_data_(axis,ptr)) /
                        (acc_time_[ptr+1] - acc_time_[ptr]);
                    double beta = acc_data_(axis,ptr) - alpha*acc_time_[ptr];
                    double t_0 = start_t_;
                    double t_1 = acc_time_[ptr+1];
                    double d_0 = alpha*acc_time_[ptr] + beta;
                    double d_1 = acc_data_(axis,ptr+1);
                    double d_v_backup = 0;
                    double d_p_backup = 0;


                    double temp_t_ratio = (start_t_ - acc_time_[ptr]) /
                        ( acc_time_[ptr+1] - acc_time_[ptr]);
                    Vec3 d_d_0_d_bw = d_acc_d_bw[axis].row(ptr+1).transpose() * temp_t_ratio +
                        d_acc_d_bw[axis].row(ptr).transpose() * (1 - temp_t_ratio);
                    Vec3 d_d_0_d_bf = d_acc_d_bf[axis].row(ptr+1).transpose() * temp_t_ratio +
                        d_acc_d_bf[axis].row(ptr).transpose() * (1 - temp_t_ratio);

                    Vec3 d_v_d_bf_backup = Vec3::Zero();
                    Vec3 d_p_d_bf_backup = Vec3::Zero();
                    Vec3 d_v_d_bw_backup = Vec3::Zero();
                    Vec3 d_p_d_bw_backup = Vec3::Zero();


                    for(size_t i = start_index; i < time.size(); ++i){
                        // Move the data pointer so that the query time is in between two data points
                        if( time.get(i) > acc_time_[0] ){
                            bool loop = true;
                            while(loop){
                                if( (time.get(i) >= acc_time_[ptr])
                                        && (time.get(i) <= acc_time_[ptr+1]) ){
                                    loop = false;
                                }else{
                                    if( ptr < (nb_acc_ - 2) ){

                                        d_p_backup = d_p_backup +  d_v_backup*(t_1 - t_0)
                                            + ((t_0 - t_1)*(t_0 - t_1)*(2.0*d_0 + d_1)/6.0);
                                        d_v_backup = d_v_backup + ((t_1 - t_0)*(d_0 + d_1)/2.0);

                                        double dt = t_1 - t_0;
                                        Vec3 d_d_1_d_bf = d_acc_d_bf[axis].row(ptr + 1).transpose();
                                        Vec3 d_d_1_d_bw = d_acc_d_bw[axis].row(ptr + 1).transpose();
                                        Vec3 temp_d_v_d_bf = dt*(d_d_0_d_bf + d_d_1_d_bf)/2.0;
                                        Vec3 temp_d_v_d_bw = dt*(d_d_0_d_bw + d_d_1_d_bw)/2.0;
                                        Vec3 temp_d_p_d_bf = dt*dt*(2.0*d_d_0_d_bf + d_d_1_d_bf)/6.0;
                                        Vec3 temp_d_p_d_bw = dt*dt*(2.0*d_d_0_d_bw + d_d_1_d_bw)/6.0;

                                        d_p_d_bf_backup = d_p_d_bf_backup + dt*d_v_d_bf_backup + temp_d_p_d_bf;
                                        d_p_d_bw_backup = d_p_d_bw_backup + dt*d_v_d_bw_backup + temp_d_p_d_bw;
                                        d_v_d_bf_backup = d_v_d_bf_backup + temp_d_v_d_bf;
                                        d_v_d_bw_backup = d_v_d_bw_backup + temp_d_v_d_bw;


                                        ptr++;
                                        t_0 = acc_time_[ptr];
                                        t_1 = acc_time_[ptr+1];
                                        d_0 = acc_data_(axis,ptr);
                                        d_1 = acc_data_(axis,ptr+1);

                                        alpha = (d_1 - d_0) / (t_1 - t_0);
                                        beta = d_0 - alpha*t_0;

                                        d_d_0_d_bf = d_acc_d_bf[axis].row(ptr).transpose();
                                        d_d_0_d_bw = d_acc_d_bw[axis].row(ptr).transpose();

                                    }else{
                                        loop=false;
                                    }
                                }
                            }
                        }

                        // Integrate
                        double time_temp = time.get(i);
                        double temp_d_1 = alpha*time_temp + beta;
                        double temp_d_v = d_v_backup +
                            ((time_temp - t_0)*(d_0 + temp_d_1 )/2.0);
                        double temp_d_p = d_p_backup + d_v_backup*(time_temp - t_0) +
                            ((t_0 - time_temp)*(t_0 - time_temp)*(2.0*d_0 + temp_d_1)/6.0);
                        double temp_d_v_var = (time_temp-start_t_)*acc_var_;
                        double temp_d_p_var = (time_temp-start_t_)*temp_d_v_var;
                        std::pair<double, double> index = time.getIndexPair(i);

                        preint[index.first][index.second].d_delta_v_d_t(axis) = (preint[index.first][index.second].delta_v(axis) - temp_d_v)/kNumDtJacobianDelta;
                        preint[index.first][index.second].d_delta_p_d_t(axis) = (preint[index.first][index.second].delta_p(axis) - temp_d_p)/kNumDtJacobianDelta;
                        preint[index.first][index.second].delta_v(axis) = temp_d_v;
                        preint[index.first][index.second].delta_p(axis) = temp_d_p;
                        preint[index.first][index.second].cov(3+axis, 3+axis) = temp_d_v_var;
                        preint[index.first][index.second].cov(6+axis, 6+axis) = temp_d_p_var;

                        // Jacobians for postintegration correction
                        temp_t_ratio = (time_temp - acc_time_[ptr]) /
                            ( acc_time_[ptr+1] - acc_time_[ptr]);
                        Vec3 d_d_1_d_bw = d_acc_d_bw[axis].row(ptr+1).transpose() * temp_t_ratio +
                            d_acc_d_bw[axis].row(ptr).transpose() * (1 - temp_t_ratio);
                        Vec3 d_d_1_d_bf = d_acc_d_bf[axis].row(ptr+1).transpose() * temp_t_ratio +
                            d_acc_d_bf[axis].row(ptr).transpose() * (1 - temp_t_ratio);

                        double dt = time_temp - t_0;
                        Vec3 temp_d_v_d_bf = dt*(d_d_0_d_bf + d_d_1_d_bf)/2.0;
                        Vec3 temp_d_v_d_bw = dt*(d_d_0_d_bw + d_d_1_d_bw)/2.0;
                        Vec3 temp_d_p_d_bf = dt*dt*(2.0*d_d_0_d_bf + d_d_1_d_bf)/6.0;
                        Vec3 temp_d_p_d_bw = dt*dt*(2.0*d_d_0_d_bw + d_d_1_d_bw)/6.0;

                        preint[index.first][index.second].d_delta_v_d_bf.row(axis) = d_v_d_bf_backup + temp_d_v_d_bf;
                        preint[index.first][index.second].d_delta_v_d_bw.row(axis) = d_v_d_bw_backup + temp_d_v_d_bw;
                        preint[index.first][index.second].d_delta_p_d_bf.row(axis) = d_p_d_bf_backup + dt*d_v_d_bf_backup + temp_d_p_d_bf;
                        preint[index.first][index.second].d_delta_p_d_bw.row(axis) = d_p_d_bw_backup + dt*d_v_d_bw_backup + temp_d_p_d_bw;


                    }
                }

            }

            void velPosPreintLPMPartial(
                        const SortIndexTracker2<double>& time,
                        std::vector<std::vector<PreintMeas> >& preint)
            {
                int data_ptr = 0;

                // For each of the query points of the timeline
                size_t start_index = 0;
                while(time.get(start_index) < start_t_){
                    start_index++;
                    if(start_index == time.size()){
                        throw std::range_error("LPM Partial: the start_time is not in the query domain");
                    }
                }
                while(acc_time_[data_ptr+1] < start_t_){
                    data_ptr++;
                    if(data_ptr == (nb_acc_-1)){
                        throw std::range_error("LPM Partial: the start_time is not in the data domain");
                    }
                }
            


                for(int axis = 0; axis < 3; ++axis)
                {
                    int ptr = data_ptr;
                    double alpha = (acc_data_(axis, ptr+1) - acc_data_(axis,ptr)) /
                        (acc_time_[ptr+1] - acc_time_[ptr]);
                    double beta = acc_data_(axis, ptr) - alpha*acc_time_[ptr];
                    double t_0 = start_t_;
                    double t_1 = acc_time_[ptr+1];
                    double d_0 = alpha*acc_time_[ptr] + beta;
                    double d_1 = acc_data_(axis,ptr+1);
                    double d_v_backup = 0;
                    double d_p_backup = 0;

                    for(size_t i = start_index; i < time.size(); ++i){
                        // Move the data pointer so that the query time is in between two data points
                        if( time.get(i) > acc_time_[0] ){
                            bool loop = true;
                            while(loop){
                                if( (time.get(i) >= acc_time_[ptr])
                                        && (time.get(i) <= acc_time_[ptr+1]) ){
                                    loop = false;
                                }else{
                                    if( ptr < (nb_acc_ - 2) ){

                                        d_p_backup = d_p_backup +  d_v_backup*(t_1 - t_0)
                                            + ((t_0 - t_1)*(t_0 - t_1)*(2.0*d_0 + d_1)/6.0);
                                        d_v_backup = d_v_backup + ((t_1 - t_0)*(d_0 + d_1)/2.0);

                                        ptr++;
                                        t_0 = acc_time_[ptr];
                                        t_1 = acc_time_[ptr+1];
                                        d_0 = acc_data_(axis,ptr);
                                        d_1 = acc_data_(axis,ptr+1);

                                        alpha = (d_1 - d_0) / (t_1 - t_0);
                                        beta = d_0 - alpha*t_0;
                                    }else{
                                        loop=false;
                                    }
                                }
                            }
                        }

                        // Integrate
                        double time_temp = time.get(i);
                        double temp_d_1 = alpha*time_temp + beta;
                        double temp_d_v = d_v_backup +
                            ((time_temp - t_0)*(d_0 + temp_d_1 )/2.0);
                        double temp_d_p = d_p_backup + d_v_backup*(time_temp - t_0) +
                            ((t_0 - time_temp)*(t_0 - time_temp)*(2.0*d_0 + temp_d_1)/6.0);
                        std::pair<double, double> index = time.getIndexPair(i);
                        preint[index.first][index.second].delta_v(axis) = temp_d_v;
                        preint[index.first][index.second].delta_p(axis) = temp_d_p;

                    }
                }
            }
        };



    // Se3 Integrator for UGPM
    class Se3Integrator
    {

        public:
            Se3Integrator(
                ImuData& imu_data,
                const double start_time,
                const PreintPrior bias_prior,
                const double window_duration,
                const double state_freq = 50.0,
                const int nb_overlap = kOverlap,
                const bool correlate = true):
                    hyper_(6),
                    correlate_(correlate),
                    state_freq_(state_freq),
                    nb_overlap_(nb_overlap),
                    start_t_(start_time)
            {

                double acc_freq = (imu_data.acc.size()-1) / (imu_data.acc.back().t - imu_data.acc[0].t);
                double gyr_freq =  (imu_data.gyr.size()-1) / (imu_data.gyr.back().t - imu_data.gyr[0].t);
                double imu_freq = std::min(acc_freq, gyr_freq);

                state_freq_ = std::max(state_freq, 5.0/(window_duration));
                state_freq_ = std::min(state_freq_, imu_freq);


                // Create the state timeline
                nb_state_ = std::ceil(window_duration*state_freq_)+(2*nb_overlap_);
                state_time_.resize(nb_state_);
                double temp_start_infer_t = start_t_ - (((double)nb_overlap_)/state_freq_);
                std::vector<double> t_vect(nb_state_);
                std::vector<double> t_vect_dt(nb_state_);
                for(int i = 0; i < nb_state_; ++i)
                {
                    state_time_(i) = temp_start_infer_t + ((double) i)/state_freq_;
                    t_vect[i] = state_time_(i);
                    t_vect_dt[i] = state_time_(i) + kNumDtJacobianDelta;
                }
                auto temp_imu_data = imu_data.get(t_vect[0] , t_vect.back());

                // Fill the private structures and other variables
                nb_gyr_ = temp_imu_data.gyr.size();
                nb_acc_ = temp_imu_data.acc.size();
                gyr_data_.resize(3, nb_gyr_);
                acc_data_.resize(3, nb_acc_);
                gyr_time_.resize(nb_gyr_);
                acc_time_.resize(nb_acc_);
                for(int i = 0; i < nb_gyr_; ++i)
                {
                    gyr_data_(0,i) = temp_imu_data.gyr[i].data[0] - bias_prior.gyr_bias[0];
                    gyr_data_(1,i) = temp_imu_data.gyr[i].data[1] - bias_prior.gyr_bias[1];
                    gyr_data_(2,i) = temp_imu_data.gyr[i].data[2] - bias_prior.gyr_bias[2];
                    gyr_time_(i) = temp_imu_data.gyr[i].t;
                }
                for(int i = 0; i < nb_acc_; ++i)
                {
                    acc_data_(0,i) = temp_imu_data.acc[i].data[0] - bias_prior.acc_bias[0];
                    acc_data_(1,i) = temp_imu_data.acc[i].data[1] - bias_prior.acc_bias[1];
                    acc_data_(2,i) = temp_imu_data.acc[i].data[2] - bias_prior.acc_bias[2];
                    acc_time_(i) = temp_imu_data.acc[i].t;
                }




                // Get prior about the preintegrated measurements from LPM preintegration
                initialiseStateWithLPM(temp_imu_data, t_vect, t_vect_dt, bias_prior);
                

                initialiseStateDiff(temp_imu_data, t_vect, t_vect_dt);

                // Initialise the hyper parameters and substract the mean to the state
                initialiseHyperParam(temp_imu_data);


                // Compute the K inv and store them for efficiency
                VecX state_std(6*nb_state_);
                state_var_.resize(6*nb_state_);
                K_inv_.resize(6, MatX(nb_state_, nb_state_));
                KK_inv_.resize(6, MatX(nb_state_, nb_state_));
                K_int_K_inv_.resize(3, MatX(nb_state_, nb_state_));
                std::vector<VecX> d_r_var(3, VecX(nb_state_));
                std::vector<VecX> acc_var(3, VecX(nb_state_));
                for(int i = 0; i < 6; ++i)
                {
                    MatX K = seKernel(state_time_, state_time_, hyper_[i].l2, hyper_[i].sf2);
                    MatX temp_to_inv(nb_state_, nb_state_);
                    temp_to_inv = K + hyper_[i].sz2*MatX::Identity(nb_state_,nb_state_);
                    K_inv_[i] = temp_to_inv.inverse();
                    KK_inv_[i] = K*K_inv_[i];
                    
                    if(i < 3)
                    {
                        MatX K_int = seKernelIntegral(start_t_, state_time_, state_time_, hyper_[i].l2, hyper_[i].sf2);
                        K_int_K_inv_[i].resize(nb_state_, nb_state_);
                        K_int_K_inv_[i] = K_int*K_inv_[i];

                        d_r_var[i] = ((-KK_inv_[i]*K).diagonal().array() + hyper_[i].sf2 + hyper_[i].sz2).matrix();
                        for(int j = 0; j < nb_state_; ++j)
                        {
                            if(d_r_var[i][j] <= 0) d_r_var[i][j] = hyper_[i].sz2;
                        }
                        if(correlate) state_std.segment(i*nb_state_,nb_state_) = d_r_var[i].array().sqrt().matrix();
                        state_var_.segment(i*nb_state_,nb_state_) = d_r_var[i];
                        d_r_var[i] = 1000.0*d_r_var[i];
                    }
                    else
                    {
                        acc_var[i-3] = ((-KK_inv_[i]*K).diagonal().array() + hyper_[i].sf2 + hyper_[i].sz2).matrix();
                        for(int j = 0; j < nb_state_; ++j)
                        {
                            if(acc_var[i-3][j] <= 0) acc_var[i-3][j] = hyper_[i].sz2;
                        }
                        if(correlate) state_std.segment(i*nb_state_,nb_state_) = acc_var[i-3].array().sqrt().matrix();
                        state_var_.segment(i*nb_state_,nb_state_) = acc_var[i-3];
                        acc_var[i-3] = 1000.0*acc_var[i-3];
                    }
                }


                // Create the cost functions and add to optimisation problem`
                std::vector< ceres::CostFunction* > cost_functions;
                ceres::Problem optimisation;
                for(int i = 0; i < 3; ++i)
                {
                    GpNormCostFunction* cost_fun = new GpNormCostFunction(KK_inv_[i], d_r_var[i]);
                    optimisation.AddResidualBlock(cost_fun, nullptr, &(state_d_r_(0,i)));
                    cost_functions.push_back(cost_fun);
                }
                RotCostFunction* rot_cost_fun = new RotCostFunction(&gyr_data_, gyr_time_, state_time_, K_inv_, start_t_, hyper_, imu_data.gyr_var);
                optimisation.AddResidualBlock(rot_cost_fun, nullptr, &(state_d_r_(0,0)), &(state_d_r_(0,1)), &(state_d_r_(0,2)));
                cost_functions.push_back(rot_cost_fun);


                AccCostFunction* acc_cost_fun = new AccCostFunction(&acc_data_, acc_time_, state_time_, K_inv_, start_t_, hyper_, imu_data.acc_var);


                // Compute the state's corelation matrix (the LPM initialisation is good enough that doing it before or after optimisation does not really change the results)
                if(correlate)
                {
                    state_cor_.resize(6*nb_state_,6*nb_state_);
                    MatX state_J(3*nb_gyr_ + 3*nb_acc_, 6*nb_state_);
                    state_J.setZero();
                    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp_J_0_r(3*nb_gyr_, nb_state_);
                    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp_J_1_r(3*nb_gyr_, nb_state_);
                    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp_J_2_r(3*nb_gyr_, nb_state_);
                    std::vector<double*> param;
                    param.push_back(&(state_d_r_(0,0)));
                    param.push_back(&(state_d_r_(0,1)));
                    param.push_back(&(state_d_r_(0,2)));
                    std::vector<double*> jacobian;
                    jacobian.push_back(temp_J_0_r.data());
                    jacobian.push_back(temp_J_1_r.data());
                    jacobian.push_back(temp_J_2_r.data());
                    VecX temp_res_r(3*nb_gyr_);
                    rot_cost_fun->Evaluate(param.data(), temp_res_r.data(), jacobian.data());
                    state_J.block(0, 0, 3*nb_gyr_,nb_state_) = temp_J_0_r;
                    state_J.block(0, nb_state_, 3*nb_gyr_,nb_state_) = temp_J_1_r;
                    state_J.block(0, 2*nb_state_, 3*nb_gyr_,nb_state_) = temp_J_2_r;

                    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp_J_0(3*nb_acc_, nb_state_);
                    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp_J_1(3*nb_acc_, nb_state_);
                    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp_J_2(3*nb_acc_, nb_state_);
                    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp_J_3(3*nb_acc_, nb_state_);
                    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp_J_4(3*nb_acc_, nb_state_);
                    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp_J_5(3*nb_acc_, nb_state_);
                    param.clear();
                    param.push_back(&(state_d_r_(0,0)));
                    param.push_back(&(state_d_r_(0,1)));
                    param.push_back(&(state_d_r_(0,2)));
                    param.push_back(&(state_acc_(0,0)));
                    param.push_back(&(state_acc_(0,1)));
                    param.push_back(&(state_acc_(0,2)));
                    jacobian.clear();
                    jacobian.push_back(temp_J_0.data());
                    jacobian.push_back(temp_J_1.data());
                    jacobian.push_back(temp_J_2.data());
                    jacobian.push_back(temp_J_3.data());
                    jacobian.push_back(temp_J_4.data());
                    jacobian.push_back(temp_J_5.data());
                    VecX temp_res(3*nb_acc_);
                    acc_cost_fun->Evaluate(param.data(), temp_res.data(), jacobian.data());
                    state_J.block(3*nb_gyr_, 0, 3*nb_acc_,nb_state_) = temp_J_0;
                    state_J.block(3*nb_gyr_, nb_state_, 3*nb_acc_,nb_state_) = temp_J_1;
                    state_J.block(3*nb_gyr_, 2*nb_state_, 3*nb_acc_,nb_state_) = temp_J_2;
                    state_J.block(3*nb_gyr_, 3*nb_state_, 3*nb_acc_,nb_state_) = temp_J_3;
                    state_J.block(3*nb_gyr_, 4*nb_state_, 3*nb_acc_,nb_state_) = temp_J_4;
                    state_J.block(3*nb_gyr_, 5*nb_state_, 3*nb_acc_,nb_state_) = temp_J_5;

                    // Launch the correlation computation in a separate thread
                    state_cor_thread_ = std::shared_ptr<std::thread>(new std::thread(&Se3Integrator::computeStateCorr, this, state_J, state_std));
                }

                // Solve the optimisation problem
                ceres::Solver::Options solver_opt;
                solver_opt.num_threads = 1;
                solver_opt.max_num_iterations = 50;

                solver_opt.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                solver_opt.function_tolerance = 1e-10;
                ceres::Solver::Summary summary;
                ceres::Solve(solver_opt, &optimisation, &summary);

                optimisation.AddResidualBlock(acc_cost_fun, nullptr, &(state_d_r_(0,0)), &(state_d_r_(0,1)), &(state_d_r_(0,2)), &(state_acc_(0,0)), &(state_acc_(0,1)), &(state_acc_(0,2)));
                for(int i = 3; i < 6; ++i)
                {
                    GpNormCostFunction* cost_fun = new GpNormCostFunction(KK_inv_[i], acc_var[i-3]);
                    optimisation.AddResidualBlock(cost_fun, nullptr, &(state_acc_(0,i-3)));
                    cost_functions.push_back(cost_fun);
                }
                cost_functions.push_back(acc_cost_fun);
                optimisation.SetParameterBlockConstant(&(state_d_r_(0,0)));
                optimisation.SetParameterBlockConstant(&(state_d_r_(0,1)));
                optimisation.SetParameterBlockConstant(&(state_d_r_(0,2)));

                solver_opt.function_tolerance = 1e-10;
                ceres::Solve(solver_opt, &optimisation, &summary);

                finishStateDiff();






                // Prepare object for later inference
                alpha_.resize(6,VecX(nb_state_));
                for(int i = 0; i < 6; ++i)
                {
                    if(i < 3)
                    {
                        alpha_[i] = K_inv_[i] * state_d_r_.col(i);
                    }
                    else
                    {
                        alpha_[i] = K_inv_[i] * state_acc_.col(i-3);
                    }
                }




                // Prepare some jacobians for the post integration correction jacobians
                d_acc_bf_.resize(3, MatX(nb_state_,3));
                d_acc_bw_.resize(3, MatX(nb_state_,3));
                d_acc_dt_.resize(3, VecX(nb_state_));
                std::vector<MatX> d_state_r_bw(3,MatX(nb_state_,3));
                MatX state_r(nb_state_,3);
                VecX dt_state = (state_time_.array() - start_t_).matrix();
                for(int axis = 0; axis < 12; ++axis)
                {
                    if(axis < 3)
                    {
                        state_r.col(axis) = K_int_K_inv_[axis]*state_d_r_.col(axis) + (dt_state*hyper_[axis].mean);
                    }
                    else if(axis < 6)
                    {
                        d_state_r_bw[0] = K_int_K_inv_[axis-3]*d_state_bw_[0];
                    }
                    else if(axis < 9)
                    {
                        d_state_r_bw[1] = K_int_K_inv_[axis-6]*d_state_bw_[1];
                    }
                    else
                    {
                        d_state_r_bw[2] = K_int_K_inv_[axis-9]*d_state_bw_[2];
                    }
                }



                // Compute the jacobians of the state 
                Vec3 start_r_dt;
                VecX temp_t(1);
                temp_t[0] = start_t_ + kNumDtJacobianDelta;
                for(int i = 0; i < 3; ++i)
                {
                    start_r_dt[i] = (seKernelIntegral(start_t_, temp_t, state_time_, hyper_[i].l2, hyper_[i].sf2) * alpha_[i])(0,0) + kNumDtJacobianDelta*hyper_[i].mean;
                    
                }
                Mat3 delta_R_dt_start = expMap(start_r_dt);

                Vec3 mean_acc(hyper_[3].mean,hyper_[4].mean,hyper_[5].mean);

                for(int i = 0; i < nb_state_; ++i)
                {
                    Mat3 temp_R = expMap(state_r.row(i));
                    d_acc_bf_[0].row(i) = temp_R.row(0);
                    d_acc_bf_[1].row(i) = temp_R.row(1);
                    d_acc_bf_[2].row(i) = temp_R.row(2);

                    Mat3 temp_d_r_bw;
                    temp_d_r_bw.row(0) = d_state_r_bw[0].row(i);
                    temp_d_r_bw.row(1) = d_state_r_bw[1].row(i);
                    temp_d_r_bw.row(2) = d_state_r_bw[2].row(i);

                    Mat3 temp_d_acc_bw = -toSkewSymMat(state_acc_.row(i).transpose() + mean_acc) * jacobianRighthandSO3(-state_r.row(i)) * temp_d_r_bw;

                    d_acc_bw_[0].row(i) = temp_d_acc_bw.row(0);
                    d_acc_bw_[1].row(i) = temp_d_acc_bw.row(1);
                    d_acc_bw_[2].row(i) = temp_d_acc_bw.row(2);

                    Vec3 acc_rot_dt = delta_R_dt_start.transpose()*(state_acc_.row(i).transpose() + mean_acc);
                    Vec3 d_acc_d_t = (acc_rot_dt - (state_acc_.row(i).transpose() + mean_acc))/kNumDtJacobianDelta;
                    (d_acc_dt_[0])[i] = d_acc_d_t(0);
                    (d_acc_dt_[1])[i] = d_acc_d_t(1);
                    (d_acc_dt_[2])[i] = d_acc_d_t(2);

                }

                if(correlate)
                {
                    state_cor_thread_->join();
                }
            }

            // Method for inference
            PreintMeas get(const double t)
            {
                PreintMeas preint;
                Vec3 r;
                Vec3 d_r_dt;
                Mat3 d_r_dw;
                Vec3 v;
                Vec3 d_v_dt;
                Mat3 d_v_dw;
                Mat3 d_v_df;
                Vec3 p;
                Vec3 d_p_dt;
                Mat3 d_p_dw;
                Mat3 d_p_df;
                VecX t_vec(1);
                t_vec[0] = t;

                double dt = t - start_t_;

                MatX state_ks(9, 6*nb_state_);
                state_ks.setZero();
                Vec9 var_vec = Vec9::Zero();
                for(int i = 0; i < 9; ++i)
                {
                    // Rotation
                    if(i<3)
                    {
                        MatX ks = seKernelIntegral(start_t_, t_vec, state_time_, hyper_[i].l2, hyper_[i].sf2);
                        r[i] =  (ks*alpha_[i] )(0,0) + (t-start_t_)*hyper_[i].mean;


                        MatX ks_K_inv = ks*K_inv_[i];
                        d_r_dw.row(i) = ks_K_inv*d_state_bw_[i];
                        state_ks.block(i, i*nb_state_, 1, nb_state_) = ks_K_inv;

                        d_r_dt[i] = (ks_K_inv*d_d_r_dt_[i] )(0,0);
                        var_vec(i) = kssInt(start_t_, t, hyper_[i].l2, hyper_[i].sf2) - (ks*K_inv_[i]*(ks.transpose()))(0,0);
                        if(var_vec(i) <= 0) var_vec(i) = dt*dt*hyper_[i].sz2;

                    }
                    // Velocity
                    else if(i < 6)
                    {
                        MatX ks = seKernelIntegral(start_t_, t_vec, state_time_, hyper_[i].l2, hyper_[i].sf2);
                        MatX ks_dt = seKernelIntegralDt(start_t_, t_vec, state_time_, hyper_[i].l2, hyper_[i].sf2);
                        v[i-3] =  (ks*alpha_[i] )(0,0) + (t-start_t_)*hyper_[i].mean;
                        d_v_dw.row(i-3) = ks*K_inv_[i]*d_acc_bw_[i-3];
                        d_v_df.row(i-3) = ks*K_inv_[i]*d_acc_bf_[i-3];
                        d_v_dt(i-3) = (ks_dt*alpha_[i])(0,0) + (ks*K_inv_[i]*d_acc_dt_[i-3])(0,0);
                        state_ks.block(i, i*nb_state_, 1, nb_state_) = ks*K_inv_[i];

                        var_vec(i) = kssInt(start_t_, t, hyper_[i].l2, hyper_[i].sf2) - (ks*K_inv_[i]*(ks.transpose()))(0,0);
                        if(var_vec(i) <= 0) var_vec(i) = std::pow(dt,2.0)*hyper_[i].sz2;
                    }
                    // Position
                    else
                    {
                        MatX ks = seKernelIntegral2(start_t_, t_vec, state_time_, hyper_[i-3].l2, hyper_[i-3].sf2);
                        MatX ks_dt = seKernelIntegral2Dt(start_t_, t_vec, state_time_, hyper_[i-3].l2, hyper_[i-3].sf2);
                        p[i-6] =  (ks*alpha_[i-3] )(0,0) + 0.5*(t-start_t_)*(t-start_t_)*hyper_[i-3].mean;
                        d_p_dw.row(i-6) = ks*K_inv_[i-3]*d_acc_bw_[i-6];
                        d_p_df.row(i-6) = ks*K_inv_[i-3]*d_acc_bf_[i-6];
                        d_p_dt(i-6) = (ks_dt*alpha_[i-3])(0,0) + (ks*K_inv_[i-3]*d_acc_dt_[i-6])(0,0);
                        state_ks.block(i, (i-3)*nb_state_, 1, nb_state_) = ks*K_inv_[i-3];
                    }
                }

                var_vec.segment<3>(6) = (state_ks.block(6,3*nb_state_,3,3*nb_state_)*(state_var_.segment(3*nb_state_, 3*nb_state_).asDiagonal())*(state_ks.block(6,3*nb_state_,3,3*nb_state_).transpose())).diagonal();
                for(int i = 6; i < 9; ++i)
                {
                    if (var_vec(i) <= 0) var_vec(i) = 0.25*std::pow(dt,4.0)*hyper_[i-3].sz2;
                }

                Mat3 j_right = jacobianRighthandSO3(r);
                preint.dt = t - start_t_;
                preint.dt_sq_half = 0.5*std::pow(preint.dt,2);
                preint.delta_R = expMap(r);
                preint.d_delta_R_d_t = j_right*d_r_dt;
                preint.d_delta_R_d_bw = j_right*d_r_dw;
                preint.delta_v = v;
                preint.d_delta_v_d_t = d_v_dt;
                preint.d_delta_v_d_bw = d_v_dw;
                preint.d_delta_v_d_bf = d_v_df;
                preint.delta_p = p;
                preint.d_delta_p_d_t = d_p_dt;
                preint.d_delta_p_d_bw = d_p_dw;
                preint.d_delta_p_d_bf = d_p_df;

                if(correlate_)
                {   
                    state_cor_mutex_.lock();
                    preint.cov = state_ks*(state_cor_)*(state_ks.transpose());
                    state_cor_mutex_.unlock();
                }
                else
                {
                    preint.cov = state_ks*(state_var_.asDiagonal())*(state_ks.transpose());
                }
                Mat9 temp_inv_std = preint.cov.diagonal().array().sqrt().inverse().matrix().asDiagonal();
                Mat9 temp_std = var_vec.array().sqrt().matrix().asDiagonal();
                Mat9 temp_d = temp_std * temp_inv_std;
                preint.cov = temp_d * preint.cov * temp_d;
                

                preint.cov.block<3,3>(0,0) = j_right * preint.cov.block<3,3>(0,0) * j_right.transpose();
                preint.cov.block<3,6>(0,3) = j_right * preint.cov.block<3,6>(0,3);
                preint.cov.block<6,3>(3,0) = preint.cov.block<3,6>(0,3).transpose();

                return preint;
            }

        private:

            std::vector<GPSeHyper> hyper_;
            bool correlate_;
            double state_freq_;
            int nb_overlap_;

            MatX state_d_r_;
            MatX state_acc_;

            std::vector<MatX> d_state_bw_;
            std::vector<VecX> d_d_r_dt_;
            MatX d_r_dt_local_;
            MatX d_r_dt_local_shift_;
            MatX delta_r_time_;
            MatX state_r_temp_;
            std::vector<MatX> delta_r_bw_;
            std::vector<MatX> d_r_bw_local_shift_;
            std::vector<MatX> d_acc_bf_;
            std::vector<MatX> d_acc_bw_;
            std::vector<VecX> d_acc_dt_;
            double start_t_;
            int nb_gyr_;
            int nb_acc_;
            int nb_state_;
            MatX gyr_data_;
            MatX acc_data_;
            VecX gyr_time_;
            VecX acc_time_;
            VecX state_time_;
            MatX state_cov_;
            MatX state_cor_;
            VecX state_var_;

            std::mutex state_cor_mutex_;
            std::shared_ptr<std::thread> state_cor_thread_;


            std::vector<VecX> alpha_;
            std::vector<MatX> KK_inv_;
            std::vector<MatX> K_inv_;
            std::vector<MatX> K_int_K_inv_;

            void initialiseStateWithLPM(ImuData& imu_data, std::vector<double>& t_vect, std::vector<double>& t_vect_dt, PreintPrior bias_prior = PreintPrior())
            {
                PreintOption preint_opt;
                preint_opt.min_freq = 500;
                preint_opt.type = LPM;
                std::vector<std::vector<double> > t;
                t.push_back(t_vect);
                t.push_back(t_vect_dt);
                t.push_back(std::vector<double>(1,start_t_));
                ImuPreintegration preint(imu_data, t_vect[0], t, preint_opt, bias_prior);


                state_d_r_.resize(nb_state_, 3);
                state_acc_.resize(nb_state_, 3);
                d_r_dt_local_.resize(3, nb_state_);
                state_r_temp_.resize(3, nb_state_);
                Mat3 start_R = preint.get(2,0).delta_R;
                std::vector<double> revolution(2, 0.0);
                std::vector<Vec3, Eigen::aligned_allocator<Vec3> > prev(2, Vec3::Zero());
                for(int i = nb_overlap_; i < nb_state_; ++i)
                {
                    for(int j = 0; j < 2; ++j)
                    {
                        Vec3 temp_r = logMap(start_R.transpose()*preint.get(j,i).delta_R);
                    
                        std::vector<Vec3> r_candidates;
                        r_candidates.push_back(addN2Pi(temp_r, revolution[j]-1));
                        r_candidates.push_back(addN2Pi(temp_r, revolution[j]));
                        r_candidates.push_back(addN2Pi(temp_r, revolution[j]+1));
                        auto [r_adjusted, id_r] = getClosest(prev[j], r_candidates);
                        prev[j] = r_adjusted;
                        revolution[j] += (id_r-1);
                    }
                    state_d_r_.row(i) = ((prev[1] - prev[0])/kNumDtJacobianDelta).transpose();
                    Vec3 temp_acc = start_R.transpose()*((preint.get(1,i).delta_v - preint.get(0,i).delta_v)/kNumDtJacobianDelta);
                    state_acc_.row(i) = temp_acc.transpose();

                    d_r_dt_local_.col(i) = jacobianRighthandSO3(prev[0]) * (state_d_r_.row(i).transpose());
                    state_r_temp_.col(i) = prev[0];
                }
                revolution[0] = 0.0;
                revolution[1] = 0.0;
                prev[0] = Vec3::Zero();
                prev[1] = Vec3::Zero();

                for(int i = (nb_overlap_ - 1); i >= 0; --i)
                {
                    for(int j = 0; j < 2; ++j)
                    {
                        Vec3 temp_r = logMap(start_R.transpose()*preint.get(j,i).delta_R);
                    
                        std::vector<Vec3> r_candidates;
                        r_candidates.push_back(addN2Pi(temp_r, revolution[j]-1));
                        r_candidates.push_back(addN2Pi(temp_r, revolution[j]));
                        r_candidates.push_back(addN2Pi(temp_r, revolution[j]+1));
                        auto [r_adjusted, id_r] = getClosest(prev[j], r_candidates);
                        prev[j] = r_adjusted;
                        revolution[j] += (id_r-1);
                    }
                    state_d_r_.row(i) = ((prev[1] - prev[0])/kNumDtJacobianDelta).transpose();
                    Vec3 temp_acc = start_R.transpose()*((preint.get(1,i).delta_v - preint.get(0,i).delta_v)/kNumDtJacobianDelta);
                    state_acc_.row(i) = temp_acc.transpose();

                    d_r_dt_local_.col(i) = jacobianRighthandSO3(prev[0]) * (state_d_r_.row(i).transpose());
                    state_r_temp_.col(i) = prev[0];
                }
            }
            void initialiseStateDiff(ImuData& imu_data, std::vector<double>& t_vect, std::vector<double>& t_vect_dt)
            {

                // Prepare for the timeshift diff
                {
                    ImuData temp_imu_data = imu_data;
                    for(size_t i = 0; i < temp_imu_data.gyr.size(); ++i) temp_imu_data.gyr[i].t -= kNumDtJacobianDelta;
                    for(size_t i = 0; i < temp_imu_data.acc.size(); ++i) temp_imu_data.acc[i].t -= kNumDtJacobianDelta;

                    d_r_dt_local_shift_.resize(3, nb_state_);
                    delta_r_time_.resize(3,nb_state_);

                    PreintOption preint_opt;
                    preint_opt.min_freq = 500;
                    preint_opt.type = LPM;
                    std::vector<std::vector<double> > t;
                    t.push_back(t_vect);
                    t.push_back(t_vect_dt);
                    t.push_back(std::vector<double>(1,start_t_));
                    PreintPrior bias_prior;
                    ImuPreintegration preint(temp_imu_data, t_vect[0], t, preint_opt, bias_prior);
                    Eigen::Matrix3d start_R = preint.get(2,0).delta_R;

                    std::array<double, 2> revolution = {0.0, 0.0};
                    std::array<Eigen::Vector3d, 3> prev = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
                    for(int i = nb_overlap_; i < nb_state_; ++i)
                    {
                        for(int j = 0; j < 2; ++j)
                        {
                            Eigen::Vector3d temp_r = logMap(start_R.transpose()*preint.get(j,i).delta_R);
                        
                            std::vector<Eigen::Vector3d> r_candidates;
                            r_candidates.push_back(addN2Pi(temp_r, revolution[j]-1));
                            r_candidates.push_back(addN2Pi(temp_r, revolution[j]));
                            r_candidates.push_back(addN2Pi(temp_r, revolution[j]+1));
                            auto [r_adjusted, id_r] = getClosest(prev[j], r_candidates);
                            prev[j] = r_adjusted;
                            revolution[j] += (id_r-1);
                        }
                        Vec3 d_r_shift = (prev[1] - prev[0])/kNumDtJacobianDelta;

                        d_r_dt_local_shift_.col(i) = jacobianRighthandSO3(prev[0]) * d_r_shift;
                        delta_r_time_.col(i) = jacobianRighthandSO3(prev[0]) * (prev[0]-(state_r_temp_.col(i)));
                    }
                    revolution = {0.0, 0.0};
                    prev = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

                    for(int i = nb_overlap_ - 1; i >= 0; --i)
                    {
                        for(int j = 0; j < 2; ++j)
                        {
                            Eigen::Vector3d temp_r = logMap(start_R.transpose()*preint.get(j,i).delta_R);
                        
                            std::vector<Eigen::Vector3d> r_candidates;
                            r_candidates.push_back(addN2Pi(temp_r, revolution[j]-1));
                            r_candidates.push_back(addN2Pi(temp_r, revolution[j]));
                            r_candidates.push_back(addN2Pi(temp_r, revolution[j]+1));
                            auto [r_adjusted, id_r] = getClosest(prev[j], r_candidates);
                            prev[j] = r_adjusted;
                            revolution[j] += (id_r-1);
                        }
                        Vec3 d_r_shift = (prev[1] - prev[0])/kNumDtJacobianDelta;

                        d_r_dt_local_shift_.col(i) = jacobianRighthandSO3(prev[0]) * d_r_shift;
                        delta_r_time_.col(i) = jacobianRighthandSO3(prev[0]) * (prev[0]-(state_r_temp_.col(i)));
                    }
                }
                // Prepare for the gyr bias diff
                {
                    d_r_bw_local_shift_.resize(3, MatX(3,nb_state_));
                    delta_r_bw_.resize(3, MatX(3,nb_state_));
                    for(int axis = 0; axis < 3; ++axis)
                    {
                        ImuData temp_imu_data = imu_data;
                        for(size_t i = 0; i < temp_imu_data.gyr.size(); ++i) temp_imu_data.gyr[i].data[axis] += kNumGyrBiasJacobianDelta;


                        PreintOption preint_opt;
                        preint_opt.min_freq = 500;
                        preint_opt.type = LPM;
                        std::vector<std::vector<double> > t;
                        t.push_back(t_vect);
                        t.push_back(t_vect_dt);
                        t.push_back(std::vector<double>(1,start_t_));
                        PreintPrior bias_prior;
                        IterativeIntegrator preint(temp_imu_data, t_vect[0], bias_prior, t, 500.0, true, true);
                        Eigen::Matrix3d start_R = preint.get(2,0).delta_R;

                        std::array<double, 2> revolution = {0.0, 0.0};
                        std::array<Eigen::Vector3d, 3> prev = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
                        for(int i = nb_overlap_; i < nb_state_; ++i)
                        {
                            for(int j = 0; j < 2; ++j)
                            {
                                Eigen::Vector3d temp_r = logMap(start_R.transpose()*preint.get(j,i).delta_R);
                            
                                std::vector<Eigen::Vector3d> r_candidates;
                                r_candidates.push_back(addN2Pi(temp_r, revolution[j]-1));
                                r_candidates.push_back(addN2Pi(temp_r, revolution[j]));
                                r_candidates.push_back(addN2Pi(temp_r, revolution[j]+1));
                                auto [r_adjusted, id_r] = getClosest(prev[j], r_candidates);
                                prev[j] = r_adjusted;
                                revolution[j] += (id_r-1);
                            }
                            Vec3 d_r_shift = (prev[1] - prev[0])/kNumDtJacobianDelta;

                            d_r_bw_local_shift_[axis].col(i) = jacobianRighthandSO3(prev[0]) * d_r_shift;
                            delta_r_bw_[axis].col(i) = jacobianRighthandSO3(prev[0]) * (prev[0]-(state_r_temp_.col(i)));
                        }
                        revolution = {0.0, 0.0};
                        prev = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

                        for(int i = nb_overlap_ - 1; i >= 0; --i)
                        {
                            for(int j = 0; j < 2; ++j)
                            {
                                Eigen::Vector3d temp_r = logMap(start_R.transpose()*preint.get(j,i).delta_R);
                            
                                std::vector<Eigen::Vector3d> r_candidates;
                                r_candidates.push_back(addN2Pi(temp_r, revolution[j]-1));
                                r_candidates.push_back(addN2Pi(temp_r, revolution[j]));
                                r_candidates.push_back(addN2Pi(temp_r, revolution[j]+1));
                                auto [r_adjusted, id_r] = getClosest(prev[j], r_candidates);
                                prev[j] = r_adjusted;
                                revolution[j] += (id_r-1);
                            }
                            Vec3 d_r_shift = (prev[1] - prev[0])/kNumDtJacobianDelta;

                            d_r_bw_local_shift_[axis].col(i) = jacobianRighthandSO3(prev[0]) * d_r_shift;
                            delta_r_bw_[axis].col(i) = jacobianRighthandSO3(prev[0]) * (prev[0]-(state_r_temp_.col(i)));
                        }
                    }
                }
                state_r_temp_.resize(0,0);
            }

            void finishStateDiff()
            {
                MatX state_r(nb_state_,3);
                VecX dt_state = (state_time_.array() - start_t_).matrix();

                for(int axis = 0; axis < 3; ++axis)
                {
                    if(axis < 3)
                    {
                        state_r.col(axis) = K_int_K_inv_[axis]*state_d_r_.col(axis) + (dt_state*hyper_[axis].mean);
                    }
                }

                d_d_r_dt_.resize(3, VecX(nb_state_));
                d_state_bw_.resize(3, MatX(nb_state_,3));
                for(int i = 0; i < nb_state_; ++i)
                {
                    Vec3 d_r = inverseJacobianRighthandSO3(state_r.row(i).transpose()) * d_r_dt_local_.col(i);


                    // For the timeshift
                    Vec3 temp_r = state_r.row(i).transpose() + inverseJacobianRighthandSO3(state_r.row(i).transpose())*delta_r_time_.col(i);
                    Vec3 d_r_dt = inverseJacobianRighthandSO3(temp_r) * d_r_dt_local_shift_.col(i);
                    Vec3 temp_d_d_r_d_t = (d_r_dt - d_r)/kNumDtJacobianDelta;
                    d_d_r_dt_[0][i] = temp_d_d_r_d_t[0];
                    d_d_r_dt_[1][i] = temp_d_d_r_d_t[1];
                    d_d_r_dt_[2][i] = temp_d_d_r_d_t[2];

                    // For the gyr bias
                    for(int axis = 0; axis < 3; ++axis)
                    {
                        Vec3 temp_r_w = state_r.row(i).transpose() + inverseJacobianRighthandSO3(state_r.row(i).transpose())*delta_r_bw_[axis].col(i);
                        Vec3 d_r_dt = inverseJacobianRighthandSO3(temp_r_w) * d_r_bw_local_shift_[axis].col(i);
                        Vec3 temp_d_d_r_d_t = (d_r_dt - d_r)/kNumGyrBiasJacobianDelta;
                        d_state_bw_[0](i,axis) = temp_d_d_r_d_t(0);
                        d_state_bw_[1](i,axis) = temp_d_d_r_d_t(1);
                        d_state_bw_[2](i,axis) = temp_d_d_r_d_t(2);

                    }
                }
            }


            void initialiseHyperParam(ImuData& imu_data)
            {
                for(int i = 0; i < 6; ++i)
                {
                    if(i < 3)
                    {
                        hyper_[i].mean = state_d_r_.col(i).mean();
                        hyper_[i].sf2 = ((state_d_r_.col(i).array() - hyper_[i].mean).square()).matrix().mean();
                        hyper_[i].sf2 = std::max(hyper_[i].sf2, imu_data.gyr_var);
                    }
                    else
                    {
                        hyper_[i].mean = state_acc_.col(i-3).mean();
                        hyper_[i].sf2 = ((state_acc_.col(i-3).array() - hyper_[i].mean).square()).matrix().mean();
                        hyper_[i].sf2 = std::max(hyper_[i].sf2, imu_data.acc_var);
                    }
                }

                Eigen::Vector3d mean_d_r(hyper_[0].mean,hyper_[1].mean,hyper_[2].mean);
                Eigen::Vector3d mean_acc(hyper_[3].mean,hyper_[4].mean,hyper_[5].mean);
                state_d_r_.rowwise() -= (mean_d_r.transpose());
                state_acc_.rowwise() -= (mean_acc.transpose());
                // Lengthscales
                double l2 = std::pow(3.0/state_freq_, 2);
                for(int i = 0; i < 3; ++i)
                {
                    hyper_[i].l2 = l2;
                    hyper_[i+3].l2 = l2;

                    hyper_[i].sz2 = imu_data.gyr_var;
                    hyper_[i+3].sz2 = imu_data.acc_var;
                }
            }

            void computeStateCorr(const MatX& state_J, const VecX& state_std)
            {
                state_cor_mutex_.lock();
                state_cor_ = state_J.transpose() * state_J;
                Eigen::LLT<MatX> cor_llt(state_cor_ + 0.00001*MatX::Identity(6*nb_state_,6*nb_state_));
                MatX L_cor = cor_llt.matrixL();
                state_cor_ = L_cor.triangularView<Eigen::Lower>().transpose().solve(L_cor.triangularView<Eigen::Lower>().solve(MatX::Identity(6*nb_state_,6*nb_state_)));


                VecX d_inv_cor = state_cor_.diagonal().array().sqrt().inverse();
                VecX temp_d_cor = state_std.array() * (d_inv_cor.array());
                state_cor_ = temp_d_cor.asDiagonal() * state_cor_ * (temp_d_cor.asDiagonal());

                state_cor_mutex_.unlock();
            }

    };





















            // Constructor given IMU data and inference timestamps
    inline ImuPreintegration::ImuPreintegration(const ImuData& imu_data,
                    const double start_t,
                    const std::vector<std::vector<double> >& infer_t,
                    const PreintOption opt,
                    const PreintPrior prior,
                    const bool rot_only,
                    const int overlap)
                        :imu_data_(imu_data)
    {
        if(!imu_data_.checkFrequency())
        {
            std::cout << "WARNING: There might be an issue with the IMU data timestamps. This is not handled in the current version (can lead to undefined behavior or alter the performance of the preintegration)." << std::endl;
        }

        // If not by chunks
        if(opt.quantum < 0)
        {
            opt_ = opt;
            prior_ = prior;
            start_t_ = start_t;


            if(opt_.type == UGPM)
            {

                // Get the maximum timestamp of infering points
                std::vector<double> temp_max;
                for(size_t i = 0; i < infer_t.size(); ++i)
                {
                    if(infer_t[i].size() > 0)
                    {
                        temp_max.push_back(*max_element(infer_t[i].begin(), infer_t[i].end()));
                    }
                }
                double duration = ( *max_element(temp_max.begin(), temp_max.end()) ) - start_t;


            
                Se3Integrator se3_int(imu_data_, start_t, prior, duration, opt_.state_freq, overlap, opt_.correlate);
                preint_.resize(infer_t.size());
                for(size_t i = 0; i < infer_t.size(); ++i)
                {
                    preint_[i].reserve(infer_t[i].size());
                    for(size_t j = 0; j < infer_t[i].size(); ++j)
                    {
                        preint_[i].push_back(se3_int.get(infer_t[i][j]));
                    }
                }
            }
            else
            {
                IterativeIntegrator integrator(imu_data_, start_t, prior, infer_t, opt_.min_freq, false, false);


                preint_.resize(infer_t.size());
                for(size_t i = 0; i < infer_t.size(); ++i)
                {
                    preint_[i].reserve(infer_t[i].size());
                    for(size_t j = 0; j < infer_t[i].size(); ++j)
                    {
                        preint_[i].push_back(integrator.get(i,j));
                    }
                }
            }
        }
        // If per-chunck
        else
        {
        
            // Get last inferrence timestamp
            std::vector<double> temp_t;
            for(auto t:infer_t)
            {
                if(t.size()>0) temp_t.push_back(t.back());
            }
            double last_t = infer_t[0].back();
            if(temp_t.size()>1)
            {
                last_t = *std::max_element(temp_t.begin(), temp_t.end());
            }


            // Get the overlap value (check diff between )
            double acc_period = (imu_data_.acc.back().t - imu_data_.acc[0].t) / (imu_data_.acc.size()-1);
            double gyr_period = (imu_data_.gyr.back().t - imu_data_.gyr[0].t) / (imu_data_.gyr.size()-1);
            double imu_period = std::max(acc_period, gyr_period);
            double t_overlap = imu_period * overlap;
            


            int nb_chuncks = (int) std::ceil((last_t - start_t)/opt.quantum);
            if(nb_chuncks == 0) nb_chuncks = 1;

            // Create pointers for the inference times
            std::vector<size_t> pointers(infer_t.size(),0);
            PreintMeas prev_chunck_preint;
            preint_.resize(infer_t.size());
            for(int i = 0; i < nb_chuncks; ++i)
            {
                double chunck_start_t = start_t + (i*opt.quantum);
                double chunck_end_t = start_t + ((i+1)*opt.quantum);
                if(i == nb_chuncks-1) chunck_end_t = std::numeric_limits<double>::infinity();

                std::vector<std::vector<double> > temp_infer_t(infer_t.size());
                if(i != (nb_chuncks - 1))
                {
                    std::vector<double> temp_temp_t;
                    temp_temp_t.push_back(chunck_end_t);
                    temp_infer_t.push_back(temp_temp_t);
                }
                for(size_t j = 0; j < infer_t.size(); ++j)
                {
                    bool loop = true;
                    while(loop)
                    {
                        if(pointers[j] < (infer_t[j].size()))
                        {
                            if(infer_t[j][pointers[j]] < chunck_end_t)
                            {
                                temp_infer_t[j].push_back(infer_t[j][pointers[j]]);
                                pointers[j]++;
                            }
                            else
                            {
                                loop = false;
                            }
                        }
                        else
                        {
                            loop = false;
                        }
                    }

                }
                auto temp_imu_data = imu_data_.get(chunck_start_t-t_overlap, chunck_end_t+t_overlap);
                auto temp_opt = opt;
                temp_opt.quantum = -1;
                ImuPreintegration preint(
                        temp_imu_data,
                        chunck_start_t,
                        temp_infer_t,
                        temp_opt,
                        prior,
                        rot_only,
                        overlap);

                if(i == 0)
                {
                    if(nb_chuncks > 1)
                    {
                        prev_chunck_preint = preint.get(infer_t.size(), 0, 0.0, 0.0);
                    }
                    for(size_t j = 0; j < infer_t.size(); ++j)
                    {
                        for(size_t h = 0; h < temp_infer_t[j].size(); ++h)
                        {
                            preint_[j].push_back(preint.get(j,h, 0.0, 0.0));
                        }
                    }
                }
                else
                {

                    for(size_t j = 0; j < infer_t.size(); ++j)
                    {
                        for(size_t h = 0; h < temp_infer_t[j].size(); ++h)
                        {
                            PreintMeas temp_preint = combinePreints(prev_chunck_preint, preint.get(j,h, 0.0, 0.0));

                            preint_[j].push_back(temp_preint);
                        }
                    }
                    if(i != (nb_chuncks-1))
                    {
                        PreintMeas temp_chunck_preint = combinePreints(prev_chunck_preint,preint.get(infer_t.size(), 0, 0.0, 0.0));

                        prev_chunck_preint = temp_chunck_preint;


                    }
                }
                
            }

        }
    }

    // Constructor overloading given only a vector of timestamps
    inline ImuPreintegration::ImuPreintegration(const ImuData& imu_data,
            const double start_t,
            const std::vector<double>& infer_t,
            const PreintOption opt,
            const PreintPrior prior,
            const bool rot_only,
            const int overlap)
                : ImuPreintegration(imu_data, start_t, std::vector<std::vector<double> >(1,infer_t), opt, prior, rot_only, overlap)
                
    {
        query_type_ = kVec;
    }

    // Constructor overloading given only a one timestamp
    inline ImuPreintegration::ImuPreintegration(const ImuData& imu_data,
            const double start_t,
            const double infer_t,
            const PreintOption opt,
            const PreintPrior prior,
            const bool rot_only,
            const int overlap)
                : ImuPreintegration(imu_data, start_t, std::vector<std::vector<double> >(1,std::vector<double>(1,infer_t)), opt, prior, rot_only, overlap)
    {
        query_type_ = kSingle;
    }


    // Get the preintegrated measurement as per indexed in the given inference times of the constructor
    inline PreintMeas ImuPreintegration::get(const int index_1, const int index_2, double acc_bias_std, double gyr_bias_std)
    {
        if ((index_1 >= 0) && 
            (index_2 >= 0) &&
            (index_1 < (int)(preint_.size())) &&
            (index_2 < (int)(preint_[index_1].size())))
        {
            // To be checked
            PreintMeas out = preint_[index_1][index_2];

            if(acc_bias_std > 0.0 || gyr_bias_std > 0.0)
            {
                Mat9_6 J;
                Mat6 b_cov = Mat6::Zero();
                b_cov.block<3,3>(0,0) = Mat3::Identity() * gyr_bias_std * gyr_bias_std;
                b_cov.block<3,3>(3,3) = Mat3::Identity() * acc_bias_std * acc_bias_std;
                J.block<3,3>(0,0) = inverseJacobianRighthandSO3(Vec3::Zero());
                J.block<3,3>(0,3) = Mat3::Zero();
                J.block<3,3>(3,0) = out.d_delta_v_d_bw;
                J.block<3,3>(3,3) = out.d_delta_v_d_bf;
                J.block<3,3>(6,0) = out.d_delta_p_d_bw;
                J.block<3,3>(6,3) = out.d_delta_p_d_bf;

                out.cov +=  J * b_cov * J.transpose();
            }
            return out;
        }
        else
        {
            throw std::range_error("ImuPreintegration::get: Trying to get precomputed preintegrated measurements (wrong index query?)");
        }

    }


    // Overload of the get function when vector in the constructor
    inline PreintMeas ImuPreintegration::get(const int index_1, double acc_bias_std, double gyr_bias_std)
    {
        if(query_type_ == kVec) return get(0, index_1, acc_bias_std, gyr_bias_std);
        else throw std::range_error("ImuPreintegration::get: The type of query does not math the type of constructor");
    }


    // Overload of the get function when single timestamp in the constructor
    inline PreintMeas ImuPreintegration::get(double acc_bias_std, double gyr_bias_std)
    {
        if(query_type_ == kSingle) return get(0, 0, acc_bias_std, gyr_bias_std);
        else throw std::range_error("ImuPreintegration::get: The type of query does not math the type of constructor");
    }


}

#endif
