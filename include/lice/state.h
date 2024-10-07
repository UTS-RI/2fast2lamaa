#pragma once

#include "lice/types.h"
#include "preint/preint.h"




class State
{

    private:
        int nb_state_;
        std::vector<double> state_time_;
        std::vector<ugpm::PreintMeas> preint_meas_;
        double state_period_;
        double start_t_;

    public:

        State(const ugpm::ImuData& imu_data, const double first_t, const double last_t, const double state_freq);


        std::vector<std::pair<Vec3, Vec3> > query(
                const std::vector<double>& query_time
                , const Vec3& acc_bias
                , const Vec3& gyr_bias
                , const Vec3& gravity
                , const Vec3& vel
                , const double dt
                ) const;

        std::vector<std::pair<Vec3, Vec3> > queryApprox(
                const std::vector<double>& query_time
                , const Vec3& acc_bias
                , const Vec3& gyr_bias
                , const Vec3& gravity
                , const Vec3& vel
                , const double dt
                ) const;

        std::pair<std::vector<std::pair<Vec3, Vec3> >,
                std::vector<std::vector<std::pair<MatX, MatX> > > >queryWthJacobian(
                const std::vector<double>& query_time
                , const Vec3& acc_bias
                , const Vec3& gyr_bias
                , const Vec3& gravity
                , const Vec3& vel
                , const double dt
                ) const;


        // Overload to query a single time
        std::pair<Vec3, Vec3> query(
                const double query_time
                , const Vec3& acc_bias
                , const Vec3& gyr_bias
                , const Vec3& gravity
                , const Vec3& vel
                , const double dt
                ) const;

        // Overload to query a single time
        std::tuple<std::pair<Vec3, Vec3>,
                std::vector<std::pair<MatX, MatX> > > queryWthJacobian(
                const double query_time
                , const Vec3& acc_bias
                , const Vec3& gyr_bias
                , const Vec3& gravity
                , const Vec3& vel
                , const double dt
                ) const;


        // Query the linear (first) and angular (second) velocity at the query time
        // WARNING: the angular velocity is only valid if the state was initialized without IMU data
        std::pair<Vec3, Vec3> queryVel(
                const double query_time
                , const Vec3& arg0
                , const Vec3& arg1
                , const Vec3& arg2
                , const Vec3& arg3
                , const double dt
                ) const;

};

void testStateMonoJacobians();


void testStateJacobians();
