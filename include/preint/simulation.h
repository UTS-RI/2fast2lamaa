#ifndef UGPM_SIMULATION_H
#define UGPM_SIMULATION_H


#include "types.h"
#include "math.h"

#include <random>
#include <iostream>

namespace ugpm
{
    struct SineProperties3{
        std::vector<double> freq = {1, 1, 1};
        std::vector<double> amp = {1, 1, 1};

        void print() const
        {
            std::cout << "Sine properties" << std::endl;
            std::cout << "  Frequencies [Hz]: "
                << freq[0] << "   "
                << freq[1] << "   "
                << freq[2] << "   "
                << std::endl;
            std::cout << "  Amplitudes [Hz]: "
                << amp[0] << "   "
                << amp[1] << "   "
                << amp[2] << "   "
                << std::endl;
        }

    };




    class RandomGenerator{
        public:
            RandomGenerator():random_eng_(std::random_device()())
            {
            }

            double randUniform(double from, double to)
            {
                std::uniform_real_distribution<double> distr(from, to);
                return distr(random_eng_);
            }
            double randGauss(double mean, double std_dev)
            {
                std::normal_distribution<double> distr(mean, std_dev);
                return distr(random_eng_);
            }

        private:

            std::default_random_engine random_eng_;
    };




    // Data structure for the IMU simulator options
    struct ImuSimulatorOption{
        // Frequency of the simulated IMU (Hz)
        double imu_frequency = 100.0;
        // Length of the dataset in second
        double dataset_length = 60.0;

        // Noise specification
        double acc_std = 0.02;
        double gyr_std = 0.002;

        // Biases (constant) FUTURE_WORK: put brownian motion on biases
        std::vector<double> acc_bias = {0, 0, 0};
        std::vector<double> gyr_bias = {0, 0, 0};

        double g_cst = 9.8;

        std::string motion_type = "fast"; // "slow" | "fast"
    };


    class ImuSimulator{

        public:
            // Constructor that computes the simulated data `imu_data_`
            // according to the given options
            ImuSimulator(ImuSimulatorOption opt)
            {
                imu_data_.t_offset = 0.0;
                imu_data_.acc_var = opt.acc_std*opt.acc_std;
                imu_data_.gyr_var = opt.gyr_std*opt.gyr_std;

                // Simulate a gravity vector in the IMU's first frame
                Vec2 g_angles = Vec2::Random();
                g_vect_ <<
                    std::sin(g_angles(0))*std::cos(g_angles(1)),
                    std::sin(g_angles(0))*std::sin(g_angles(1)),
                    std::cos(g_angles(0));
                g_vect_ *= opt.g_cst;


                // Pick trajectory characteristics depending on the simulator's option
                int number_sines = 3;
                double vel_amplitude_min;
                double vel_amplitude_max;
                double vel_freq_min;
                double vel_freq_max;
                double rot_amplitude_min;
                double rot_amplitude_max;
                double rot_freq_min;
                double rot_freq_max;
                if(opt.motion_type == "slow")
                {
                    vel_amplitude_min = 0.3;
                    vel_amplitude_max = 2;
                    vel_freq_min = 0.05;
                    vel_freq_max = 0.1;
                    rot_amplitude_min = 0.5;
                    rot_amplitude_max = 0.75;
                    rot_freq_min = 0.05;
                    rot_freq_max = 0.3;
                }
                else if(opt.motion_type == "fast")
                {
                    vel_amplitude_min = 3;
                    vel_amplitude_max = 5;
                    vel_freq_min = 0.5;
                    vel_freq_max = 1.5;
                    rot_amplitude_min = 0.5;
                    rot_amplitude_max = 1.0;
                    rot_freq_min = 0.5;
                    rot_freq_max = 0.6;

                }
                else if(opt.motion_type == "all")
                {
                    vel_amplitude_min = 0.3;
                    vel_amplitude_max = 5;
                    vel_freq_min = 0.05;
                    vel_freq_max = 1.5;
                    rot_amplitude_min = 0.5;
                    rot_amplitude_max = 1.0;
                    rot_freq_min = 0.05;
                    rot_freq_max = 0.6;

                }
                else
                {
                    throw std::range_error("Unkown 'motion_type' given to the IMU simulator");
                }
                


                // Creating the sine properties from the above characteristics
                // The velocities are the combination of multiple random sines
                // that rougthly cover the spectrum of the characteristics
                // The rotations are based on one sine only (simpler math ^^)
                double vel_freq_quantum = vel_freq_max - vel_freq_min;
                vel_sine_.resize(number_sines); 
                for(int axis = 0; axis < 3; ++axis)
                {
                    rot_sine_.amp[axis] = rand_gen_.randUniform(rot_amplitude_min, rot_amplitude_max);
                    rot_sine_.freq[axis] = rand_gen_.randUniform(rot_freq_min, rot_freq_max);
                    for(int i = 0; i < number_sines; ++i)
                    {
                        double temp_vel_freq_min = vel_freq_min + i*vel_freq_quantum;
                        double temp_vel_freq_max = vel_freq_min + (i+1)*vel_freq_quantum;
                        vel_sine_[i].amp[axis] = rand_gen_.randUniform(vel_amplitude_min, vel_amplitude_max);
                        vel_sine_[i].freq[axis] = rand_gen_.randUniform(temp_vel_freq_min, temp_vel_freq_max);
                    }

                }


                // Create the simulated IMU measurements with the specified noise
                int nb_samples = opt.imu_frequency*opt.dataset_length;
                imu_data_.acc.resize(nb_samples);
                imu_data_.gyr.resize(nb_samples);
                for(int i = 0; i < nb_samples; ++i)
                {
                    double t = i*opt.dataset_length/nb_samples;
                    
                    auto acc = stdToVec3(getAcc(t));
                    auto eul = getEul(t);
                    auto ang_vel = getAngVel(t);
                    Vec3 temp_acc = (eulToRotMat(getEul(t))).transpose() * (acc - g_vect_);
                    for(int axis = 0; axis < 3; ++axis)
                    {
                        imu_data_.acc[i].t = t;
                        imu_data_.acc[i].data[axis] = temp_acc(axis) + rand_gen_.randGauss(0, opt.acc_std);
                        imu_data_.gyr[i].t = t;
                        imu_data_.gyr[i].data[axis] = ang_vel[axis] + rand_gen_.randGauss(0, opt.gyr_std);
                    }

                }
            }

            // Get the simulated IMU data between the given times (in seconds)
            ImuData get(double from, double to)
            {
                // Check if the query interval makes sense
                if(from < to)
                {
                    return imu_data_.get(from, to);
                }
                // If the query inteval does not make sense throw an exception
                else
                {
                    throw std::invalid_argument("The argument of ImuReader::Get are not consistent");
                }
            }


            // Test a preintegrated measurements against the ground truth
            std::vector<double> testPreint(double start, double end, PreintMeas preint)
            {
                std::vector<double> output;


                Mat3 rot_start_tr = eulToRotMat(getEul(start)).transpose();
                output.push_back( logMap(preint.delta_R.transpose()*rot_start_tr*eulToRotMat(getEul(end)) ).norm() );
                
                double dt = end - start;
                output.push_back( (rot_start_tr*( stdToVec3(getVel(end)) - stdToVec3(getVel(start)) - (dt*g_vect_) ) - preint.delta_v).norm());
                output.push_back( (rot_start_tr*( stdToVec3(getPos(end)) - stdToVec3(getPos(start)) - (dt*stdToVec3(getVel(start))) -(0.5*dt*dt*g_vect_) ) - preint.delta_p).norm());
                return output;
            }

            Vec9 preintErrorVec(double start, double end, PreintMeas preint)
            {
                Mat3 rot_start_tr = eulToRotMat(getEul(start)).transpose();

                Vec9 residual;
                residual.segment<3>(0) = logMap(preint.delta_R.transpose()*rot_start_tr*eulToRotMat(getEul(end)) );

                
                double dt = end - start;
                residual.segment<3>(3) = (rot_start_tr*( stdToVec3(getVel(end)) - stdToVec3(getVel(start)) - (dt*g_vect_) ) - preint.delta_v);
                residual.segment<3>(6) = (rot_start_tr*( stdToVec3(getPos(end)) - stdToVec3(getPos(start)) - (dt*stdToVec3(getVel(start))) -(0.5*dt*dt*g_vect_) ) - preint.delta_p);

                return residual;
            }

            double testPreintCov(double start, double end, PreintMeas preint)
            {
                double output;

                Mat3 rot_start_tr = eulToRotMat(getEul(start)).transpose();

                Vec9 residual;
                residual.segment<3>(0) = logMap(preint.delta_R.transpose()*rot_start_tr*eulToRotMat(getEul(end)) );

                
                double dt = end - start;
                residual.segment<3>(3) = (rot_start_tr*( stdToVec3(getVel(end)) - stdToVec3(getVel(start)) - (dt*g_vect_) ) - preint.delta_v);
                residual.segment<3>(6) = (rot_start_tr*( stdToVec3(getPos(end)) - stdToVec3(getPos(start)) - (dt*stdToVec3(getVel(start))) -(0.5*dt*dt*g_vect_) ) - preint.delta_p);

                output = residual.transpose() * (preint.cov.inverse()) * residual;

                return output;
            }

            // Get average velocity (at IMU timestamps)
            double getAvgVel()
            {
                double avg_vel = 0;
                int nb_samples = imu_data_.acc.size();
                double coeff = (imu_data_.acc.back().t - imu_data_.acc[0].t ) / double(nb_samples-1.0); 
                for(int i = 0; i < nb_samples; ++i)
                {
                    double t = i*coeff;
                    auto vel = getVel(t);
                    Eigen::Map<Vec3> vel_temp(&(vel[0]));
                    avg_vel += vel_temp.norm();

                }
                avg_vel /= double(nb_samples);
                return avg_vel;
            }

            // Get average angular velocity (at IMU timestamps)
            double getAvgAngVel()
            {
                double avg_vel = 0;
                int nb_samples = imu_data_.acc.size();
                double coeff = (imu_data_.acc.back().t - imu_data_.acc[0].t ) / double(nb_samples-1.0); 
                for(int i = 0; i < nb_samples; ++i)
                {
                    double t = i*coeff;
                    auto vel = getAngVel(t);
                    Eigen::Map<Vec3> vel_temp(&(vel[0]));
                    avg_vel += vel_temp.norm();

                }
                avg_vel /= double(nb_samples);
                return avg_vel;
            }

            // Get the trajectory length (numerically)
            double getTranslationDistance(const double from, const double to)
            {
                double output = 0;
                Vec3 prev_pos = stdToVec3(getPos(from));
                for(double t = (from+kNumericalQuantum); t < to; t += kNumericalQuantum)
                {
                    Vec3 temp_pos = stdToVec3(getPos(t));
                    output += (temp_pos-prev_pos).norm();
                    prev_pos = temp_pos;
                }
                output += (stdToVec3(getPos(to))-prev_pos).norm();

                return output;
            }


            double getOrientationDistance(const double from, const double to)
            {
                double output = 0;

                Mat3 prev_rot = eulToRotMat(getEul(from));
                for(double t = (from+kNumericalQuantum); t < to; t += kNumericalQuantum)
                {
                    Mat3 temp_rot = eulToRotMat(getEul(t));
                    output += (logMap(temp_rot.transpose() * prev_rot)).norm();
                    prev_rot = temp_rot;
                }
                output += (logMap(eulToRotMat(getEul(to)).transpose() * prev_rot)).norm();

                return output;
            }


        private:
            // Stored the simulated data
            ImuData imu_data_;

            double kNumericalQuantum = 0.0001;

            // Store the trajectory parameters
            std::vector<SineProperties3> vel_sine_;
            SineProperties3 rot_sine_;
            Vec3 g_vect_;

            // For the sake of potential visualisation
            std::vector<double> vel_offset_ = {0, 0, 0};

            // For random number generation
            RandomGenerator rand_gen_;

            // Get the position values based on simulated sines
            std::vector<double> getPos(const double t)
            {
                // Initialise the output vector
                std::vector<double> output = vel_offset_;
                for(auto& p : output) p*=t;

                // Apply the simulation model
                for(int i = 0; i < vel_sine_.size(); ++i)
                {
                    for(int j = 0; j < 3; ++j)
                    {
                        output[j] += vel_sine_[i].amp[j]*sin(2*M_PI*vel_sine_[i].freq[j]*t)/(2*M_PI*vel_sine_[i].freq[j]);
                    }
                }
                return output;
            }




            // Get the velocity values based on simulated sines
            std::vector<double> getVel(const double t)
            {
                // Initialise the output vector
                std::vector<double> output = vel_offset_;

                // Apply the simulation model
                for(int i = 0; i < vel_sine_.size(); ++i)
                {
                    for(int j = 0; j < 3; ++j)
                    {
                        output[j] += vel_sine_[i].amp[j]*cos(2*M_PI*vel_sine_[i].freq[j]*t);
                    }
                }
                return output;
            }




            // Get the acceleration values based on simulated sines
            std::vector<double> getAcc(const double t)
            {
                // Initialise the output vector
                std::vector<double> output = {0, 0, 0};

                // Apply the simulation model
                for(int i = 0; i < vel_sine_.size(); ++i)
                {
                    for(int j = 0; j < 3; ++j)
                    {
                        output[j] -= vel_sine_[i].amp[j]*sin(2*M_PI*vel_sine_[i].freq[j]*t)*(2*M_PI*vel_sine_[i].freq[j]);
                    }
                }
                return output;
            }




            // Get the euler angle representation of orientation based on simulated sines
            std::vector<double> getEul(const double t)
            {
                // Initialise the output vector
                std::vector<double> output = {0, 0, 0};
                // Simulate sine on orientation
                for(int j = 0; j < 3; ++j)
                {
                    output[j] += rot_sine_.amp[j]*sin(2*M_PI*rot_sine_.freq[j]*t);
                }
                return output;
            }





            // Get angular velocity based on simulated sines
            std::vector<double> getAngVel(const double t)
            {
                // Initialise the output vector
                std::vector<double> output(3);
                // Sinulate from euler sine
                output[0] = 2*rot_sine_.amp[2]*rot_sine_.freq[2]*M_PI*cos(2*M_PI*rot_sine_.freq[2]*t) - 2*rot_sine_.amp[0]*rot_sine_.freq[0]*M_PI*sin(rot_sine_.amp[1]*sin(2*M_PI*rot_sine_.freq[1]*t))*cos(2*M_PI*rot_sine_.freq[0]*t);
                output[1] = 2*rot_sine_.amp[1]*rot_sine_.freq[1]*M_PI*cos(2*M_PI*rot_sine_.freq[1]*t)*cos(rot_sine_.amp[2]*sin(2*M_PI*rot_sine_.freq[2]*t)) + 2*rot_sine_.amp[0]*rot_sine_.freq[0]*M_PI*sin(rot_sine_.amp[2]*sin(2*M_PI*rot_sine_.freq[2]*t))*cos(2*M_PI*rot_sine_.freq[0]*t)*cos(rot_sine_.amp[1]*sin(2*M_PI*rot_sine_.freq[1]*t));
                output[2] = 2*rot_sine_.amp[0]*rot_sine_.freq[0]*M_PI*cos(2*M_PI*rot_sine_.freq[0]*t)*cos(rot_sine_.amp[1]*sin(2*M_PI*rot_sine_.freq[1]*t))*cos(rot_sine_.amp[2]*sin(2*M_PI*rot_sine_.freq[2]*t)) - 2*rot_sine_.amp[1]*rot_sine_.freq[1]*M_PI*sin(rot_sine_.amp[2]*sin(2*M_PI*rot_sine_.freq[2]*t))*cos(2*M_PI*rot_sine_.freq[1]*t);

                return output;
            }
    };


}





#endif
