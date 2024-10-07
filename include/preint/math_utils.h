#ifndef UGPM_MATH_H
#define UGPM_MATH_H

#include "types.h"
#include "Eigen/unsupported/SpecialFunctions"
#include "Eigen/Geometry"

namespace ugpm
{

    const double kExpNormTolerance = 1e-14;
    const double kLogTraceTolerance = 3.0 - kExpNormTolerance;


    const double kNumDtJacobianDelta = 0.01;
    const double kNumAccBiasJacobianDelta = 0.0001;
    const double kNumGyrBiasJacobianDelta = 0.0001;

    const double kSqrt2 = std::sqrt(2.0);
    const double kSqrtPi = std::sqrt(M_PI);

    inline Mat3 eulToRotMat(double eul_z, double eul_y, double eul_x)
    {

        Mat3 transform;
        double c1 = std::cos(eul_x);
        double c2 = std::cos(eul_y);
        double c3 = std::cos(eul_z);
        double s1 = std::sin(eul_x);
        double s2 = std::sin(eul_y);
        double s3 = std::sin(eul_z);
        transform << c1*c2, c1*s2*s3 - c3*s1, s1*s3 + c1*c3*s2,
                  c2*s1, c1*c3 + s1*s2*s3, c3*s1*s2 - c1*s3,
                -s2, c2*s3, c2*c3;
    
        return transform;
    }
    inline Mat3 eulToRotMat(std::vector<double> eul)
    {
        if(eul.size() != 3) throw std::range_error("Wrong vector size for Euler to Rotation matrix conversion");
        return eulToRotMat(eul[2], eul[1], eul[0]);
    }




    // SO3 Log mapping
    inline Vec3 logMap(const Mat3& rot_mat){
        Eigen::AngleAxisd rot_axis(rot_mat);
        return rot_axis.angle() * rot_axis.axis();
    }   


    // SO3 Exp mapping
    inline Mat3 expMap(const Vec3& vec){
        Eigen::AngleAxisd rot_axis(vec.norm(), vec.normalized());
        return rot_axis.toRotationMatrix();
    }



    // Righthand Jacobian of SO3 Exp mapping
    inline Mat3 jacobianRighthandSO3( const Vec3& rot_vec)
    {
        Mat3 output = Mat3::Identity();
        double vec_norm = rot_vec.norm();
        
        Vec3 vec = rot_vec;

        if(vec_norm > kExpNormTolerance)
        {
            Mat3 skew_mat;
            skew_mat << 0.0, -vec(2), vec(1),
                        vec(2), 0.0, -vec(0),
                        -vec(1), vec(0), 0.0;
            
            output += ( (vec_norm - sin(vec_norm)) / (vec_norm*vec_norm*vec_norm) )*skew_mat*skew_mat  - ( (1.0 - cos(vec_norm))/(vec_norm*vec_norm) )*skew_mat;
        }
        return output;
    }

    // Inverse Righthand Jacobian of SO3 Exp mapping
    inline Mat3 inverseJacobianRighthandSO3( Vec3 rot_vec)
    {
        Mat3 output = Mat3::Identity();
        double vec_norm = rot_vec.norm();
        
        Vec3 vec = rot_vec;
        if(vec_norm > kExpNormTolerance)
        {
            Mat3 skew_mat;
            skew_mat << 0.0, -vec(2), vec(1),
                        vec(2), 0.0, -vec(0),
                        -vec(1), vec(0), 0.0;
            
            output += 0.5*skew_mat + ( ( (1.0/(vec_norm*vec_norm)) - ((1+std::cos(vec_norm))/(2.0*vec_norm*std::sin(vec_norm))) )*skew_mat*skew_mat);
        }
        return output;
    }


    inline MatX seKernel(const VecX& x1, const VecX& x2, const double l2, const double sf2)
    {
        MatX D2(x1.size(), x2.size());
        for(int i = 0; i < x2.size(); i++)
        {
            D2.col(i) = (x1.array() - x2(i)).square();
        }
        return ((D2 * (-0.5/l2)).array().exp() * sf2).matrix();
    }



    inline MatX seKernelIntegral(const double a, const VecX& b, const VecX& x2, const double l2, const double sf2)
    {
        double sqrt_inv_l2 = std::sqrt(1.0/l2);
        double alpha = kSqrt2*sf2*kSqrtPi / (2.0*sqrt_inv_l2);

        MatX A(b.size(), x2.size());
        RowX c = (kSqrt2*(-x2.transpose().array()+a)*sqrt_inv_l2/2.0).erf().matrix();
        for(int i = 0; i < x2.size(); i++)
        {
            A.col(i) = (kSqrt2*(b.array()-x2(i))*sqrt_inv_l2/2.0).array().erf().matrix();
        }
        return alpha*( A.rowwise() - c);
    }



    inline MatX seKernelIntegralDt(const double a, const VecX& b, const VecX& x2, const double l2, const double sf2)
    {
        MatX A(b.size(), x2.size());
        RowX c = sf2*((x2.transpose().array() - a).square()/(-2.0*l2)).exp().matrix();
        for(int i = 0; i < x2.size(); i++)
        {
            A.col(i) = sf2*((b.array() - x2(i)).pow(2)/(-2.0*l2)).exp();
        }
        MatX out(b.size(), x2.size());
        return A.rowwise() - c;
    }



    inline MatX seKernelIntegral2(const double a, const VecX& b, const VecX& x2, const double l2, const double sf2)
    {
        double sqrt_inv_l2 = std::sqrt(1.0/l2);

        RowX a_x2 = (-x2.transpose().array()+a).matrix();
        RowX a_x2_erf = (kSqrt2*(a_x2)*sqrt_inv_l2/2.0).array().erf().matrix();
        RowX c =  (  kSqrt2*(a_x2.array().square()/(-2.0*l2)).exp()/(kSqrtPi*sqrt_inv_l2)
                +   a_x2_erf.array()*(a_x2.array()) ).matrix();
        MatX A(b.size(), x2.size());
        for(int i = 0; i < x2.size(); i++)
        {
            VecX b_x2 = (b.array()-x2(i)).matrix();

            A.col(i) = (a_x2_erf(i)*(-b.array()+a)
                        +   (kSqrt2*(b_x2)*sqrt_inv_l2/2.0).array().erf()*((b_x2).array())
                        +   kSqrt2*(b_x2.array().square()/(-2.0*l2)).exp()/(kSqrtPi*sqrt_inv_l2)).matrix();
        }
        double alpha = kSqrt2*sf2*kSqrtPi/(2.0*sqrt_inv_l2);
        return alpha*(A.matrix().rowwise() - c
                 ).matrix();
    }



    inline MatX seKernelIntegral2Dt(const double a, const VecX& b, const VecX& x2, const double l2, const double sf2)
    {
        double sqrt_inv_l2 = std::sqrt(1.0/l2);
        RowX a_x2 = (-x2.transpose().array()+a).matrix();
        RowX a_x2_exp = ( a_x2.array().square()/(-2.0*l2)).exp();
        RowX a_x2_erf = (kSqrt2*(a_x2)*sqrt_inv_l2/2.0).array().erf().matrix();
        MatX A(b.size(), x2.size());
        for(int i = 0; i < x2.size(); i++)
        {
            A.col(i) = (( (( -kSqrt2*(b.array()-x2(i))*sqrt_inv_l2*0.5).erf() + a_x2_erf(i))*kSqrt2*sf2*kSqrtPi 
                +2.0*sqrt_inv_l2*sf2*(b.array()-a)*a_x2_exp(i)
                )/(-2.0*sqrt_inv_l2)).matrix();
        }
        return A;
    }



    inline Mat3 toSkewSymMat(const Vec3& rot_vec)
    {
        Mat3 skew_mat;
        skew_mat << 0.0, -rot_vec(2), rot_vec(1),
                    rot_vec(2), 0.0, -rot_vec(0),
                    -rot_vec(1), rot_vec(0), 0.0;
        return skew_mat;

    }

    inline Row9 mat3ToRow(Mat3 R)
    {
        Row9 output;
        output = Eigen::Map<Row9>(R.data());
        return output;
    }



    inline Vec9 jacobianExpMapZeroV(const Vec3& v)
    {
        Vec9 output;
        output << 0, v(2), -v(1), -v(2), 0, v(0), v(1), -v(0), 0;
        return output;
    }
    inline Mat9_3 jacobianExpMapZeroM(const Mat3& M)
    {
        Mat9_3 output;
        output << 0, 0, 0,
                M(2,0), M(2,1), M(2,2),
                -M(1,0), -M(1,1), -M(1,2),
                -M(2,0), -M(2,1), -M(2,2),
                0, 0, 0,
                M(0,0), M(0,1), M(0,2),
                M(1,0), M(1,1), M(1,2),
                -M(0,0), -M(0,1), -M(0,2),
                0, 0, 0;
        return output;
    }

    inline Mat3_9 jacobianLogMap(const Mat3& rot_mat)
    {

        Mat3_9 output;
        double trace_mat = rot_mat.trace();
        
        if(trace_mat < kLogTraceTolerance){

            // Equation from MATLAB symbolic toolbox (might have a better formualtion, to inspect later)
            output(0,0) = - (rot_mat(1,2) - rot_mat(2,1))/(4*(std::pow(rot_mat(0,0)/2.0
                            + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2) - 1)) - 
                            (std::acos(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5)*
                             (rot_mat(1,2) - rot_mat(2,1))*(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0
                                 + rot_mat(2,2)/2.0 - 0.5))/(4*std::pow(1 - std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 
                                     + rot_mat(2,2)/2.0 - 0.5,2),1.5));
            output(0,1) = 0.0;
            output(0,2) = 0.0;
            output(0,3) = 0.0;
            output(0,4) = - (rot_mat(1,2) - rot_mat(2,1))/
                (4*(std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2) - 1)) -
                (std::acos(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5)*
                 (rot_mat(1,2) - rot_mat(2,1))*(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5))/
                (4*std::pow(1 - std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2),1.5));
            output(0,5) = std::acos(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5)
                /(2*std::pow(1 - std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2),0.5));
            output(0,6) = 0.0;
            output(0,7) = -std::acos(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5)/
                (2*std::pow(1 - std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2),0.5));
            output(0,8) = - (rot_mat(1,2) - rot_mat(2,1))/
                (4*(std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2) - 1)) -
                (std::acos(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5)*
                 (rot_mat(1,2) - rot_mat(2,1))*(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5))/
                (4*std::pow(1 - std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2),1.5));
            output(1,0) = (rot_mat(0,2) - rot_mat(2,0))/
                (4*(std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2) - 1)) + 
                (std::acos(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5)*
                 (rot_mat(0,2) - rot_mat(2,0))*(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5))/
                (4*std::pow(1 - std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2),1.5));
            output(1,1) = 0.0;
            output(1,2) = -std::acos(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5)/
                (2*std::pow(1 - std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2),0.5));
            output(1,3) = 0.0;
            output(1,4) = (rot_mat(0,2) - rot_mat(2,0))/
                (4*(std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2) - 1)) + 
                (std::acos(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5)*
                 (rot_mat(0,2) - rot_mat(2,0))*(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5))/(
                 4*std::pow(1 - std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2),1.5));
            output(1,5) = 0.0;
            output(1,6) = std::acos(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5)/
                (2*std::pow(1 - std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2),0.5));
            output(1,7) = 0.0;
            output(1,8) = (rot_mat(0,2) - rot_mat(2,0))/
                (4*(std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2) - 1)) + 
                (std::acos(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5)*(rot_mat(0,2) - rot_mat(2,0))*
                 (rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5))/
                (4*std::pow(1 - std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2),1.5));
            output(2,0) = - (rot_mat(0,1) - rot_mat(1,0))/
                (4*(std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2) - 1)) - 
                (std::acos(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5)*(rot_mat(0,1) - rot_mat(1,0))*
                 (rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5))/
                (4*std::pow(1 - std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2),1.5));
            output(2,1) = std::acos(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5)/
                (2*std::pow(1 - std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2),0.5));
            output(2,2) = 0.0;
            output(2,3) = -std::acos(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5)/
                (2*std::pow(1 - std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2),0.5));
            output(2,4) = - (rot_mat(0,1) - rot_mat(1,0))/
                (4*(std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2) - 1)) - 
                (std::acos(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5)*(rot_mat(0,1) - rot_mat(1,0))*
                 (rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5))/
                (4*std::pow(1 - std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2),1.5));
            output(2,5) = 0.0;
            output(2,6) = 0.0;
            output(2,7) = 0.0;
            output(2,8) = - (rot_mat(0,1) - rot_mat(1,0))/
                (4*(std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2) - 1)) - 
                (std::acos(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5)*(rot_mat(0,1) - rot_mat(1,0))*
                 (rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5))/
                (4*std::pow(1 - std::pow(rot_mat(0,0)/2.0 + rot_mat(1,1)/2.0 + rot_mat(2,2)/2.0 - 0.5,2),1.5));
        }else{
            output <<   0,   0,    0,    0, 0, 0.5,   0, -0.5, 0,
                        0,   0, -0.5,    0, 0,   0, 0.5,    0, 0,
                        0, 0.5,    0, -0.5, 0,   0,   0,    0, 0;
        }

        return output;
    }


    inline Mat3_9 jacobianXv(const Vec3& v)
    {
        Mat3_9 output;
        output <<
            v[0]   , 0.0    , 0.0    , v[1]   , 0.0    , 0.0    , v[2]   , 0.0    , 0.0,
            0.0    , v[0]   , 0.0    , 0.0    , v[1]   , 0.0    , 0.0    , v[2]   , 0.0,
            0.0    , 0.0    , v[0]   , 0.0    , 0.0    , v[1]   , 0.0    , 0.0    , v[2];
        return output;
    }

    inline Mat9 jacobianTranspose()
    {
        Mat9 output;
        output <<
            1.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0,
            0.0    , 0.0    , 0.0    , 1.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0,
            0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 1.0    , 0.0    , 0.0,
            0.0    , 1.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0,
            0.0    , 0.0    , 0.0    , 0.0    , 1.0    , 0.0    , 0.0    , 0.0    , 0.0,
            0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 1.0    , 0.0,
            0.0    , 0.0    , 1.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0,
            0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 1.0    , 0.0    , 0.0    , 0.0,
            0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 1.0;
        return output;
    }

    inline Mat9 jacobianYX(const Mat3& Y)
    {
        Mat9 output;
        output = Mat9::Zero();
        output.block<3,3>(0,0) = Y;
        output.block<3,3>(3,3) = Y;
        output.block<3,3>(6,6) = Y;
        return output;
    }
    inline Mat3_9 jacobianYXv(const Mat3& Y, const Vec3& v)
    {
        Mat3_9 output;
        output <<
            mat3ToRow((v * Y.row(0)).transpose()) ,
            mat3ToRow((v * Y.row(1)).transpose()) ,
            mat3ToRow((v * Y.row(2)).transpose());
        return output;
    }

    inline Mat9 jacobianYXW(const Mat3& Y, const Mat3& W)
    {
        Mat9 output;
        output <<
            mat3ToRow((W.col(0) * Y.row(0)).transpose()) ,
            mat3ToRow((W.col(0) * Y.row(1)).transpose()) ,
            mat3ToRow((W.col(0) * Y.row(2)).transpose()) ,
            mat3ToRow((W.col(1) * Y.row(0)).transpose()) ,
            mat3ToRow((W.col(1) * Y.row(1)).transpose()) ,
            mat3ToRow((W.col(1) * Y.row(2)).transpose()) ,
            mat3ToRow((W.col(2) * Y.row(0)).transpose()) ,
            mat3ToRow((W.col(2) * Y.row(1)).transpose()) ,
            mat3ToRow((W.col(2) * Y.row(2)).transpose());
        return output;
    }


    inline double kssInt(const double a, const double b, const double l2, const double sf2)
    {
        return 2.0*l2*sf2*std::exp(-std::pow(a - b,2)/(2.0*l2)) - 2.0*l2*sf2 + (std::sqrt(2.0)*sf2*std::sqrt(M_PI)*std::erf((std::sqrt(2.0)*(a - b)*std::sqrt(1.0/l2))/2.0)*(a - b))/std::sqrt(1.0/l2);

    }


    inline Vec3 addN2Pi(const Vec3& r, const int n)
    {
        double norm_r = r.norm();
        if(norm_r != 0)
        {
            Vec3 unit_r = r/norm_r;
            return unit_r*(2.0*M_PI*n + norm_r);
        }
        else
        {
            return r;
        }
    }

    inline std::pair<Vec3, int> getClosest(const Vec3& t, const std::vector<Vec3> s)
    {
        int id_min = 0;
        double dist_min = std::numeric_limits<double>::max();
        for(size_t i = 0; i < s.size(); ++i)
        {
            if((t - s[i]).norm() < dist_min)
            {
                dist_min = (t - s[i]).norm();
                id_min = i;
            }
        }
        return {s[id_min], id_min};
    }


    inline MatX reprojectAccData(
                const std::vector<PreintMeas>& preint,
                const MatX& acc )
    {
        MatX output(3, acc.cols());

        for(int i = 0; i < acc.cols(); ++i)
        {
            output.col(i) = preint[i].delta_R * acc.col(i);
        }
        return output;
    }

    inline MatX reprojectAccData(
                const std::vector<PreintMeas>& preint,
                const MatX& acc,
                const Mat3& delta_R_dt_start,
                std::vector<MatX>& d_acc_d_bf,
                std::vector<MatX>& d_acc_d_bw,
                std::vector<VecX>& d_acc_d_dt
                )
    {
        MatX output(3, acc.cols());
        for(int i = 0; i < 3; ++i)
        {
            d_acc_d_bf.push_back(MatX(acc.cols(),3));
            d_acc_d_bw.push_back(MatX(acc.cols(),3));
            d_acc_d_dt.push_back(VecX(acc.cols()));
        }

        for(int i = 0; i < acc.cols(); ++i)
        {
            Vec3 temp_acc = acc.col(i);


            d_acc_d_bf[0].row(i) = preint[i].delta_R.row(0);
            d_acc_d_bf[1].row(i) = preint[i].delta_R.row(1);
            d_acc_d_bf[2].row(i) = preint[i].delta_R.row(2);



            Mat9_3 temp_d_R_d_bw = jacobianExpMapZeroM(preint[i].d_delta_R_d_bw);
            Row9 temp_1;
            temp_1 <<   preint[i].delta_R(0,0)*temp_acc(0),preint[i].delta_R(0,1)*temp_acc(0),preint[i].delta_R(0,2)*temp_acc(0),
                        preint[i].delta_R(0,0)*temp_acc(1),preint[i].delta_R(0,1)*temp_acc(1),preint[i].delta_R(0,2)*temp_acc(1),
                        preint[i].delta_R(0,0)*temp_acc(2),preint[i].delta_R(0,1)*temp_acc(2),preint[i].delta_R(0,2)*temp_acc(2);
            Row9 temp_2;
            temp_2 <<   preint[i].delta_R(1,0)*temp_acc(0),preint[i].delta_R(1,1)*temp_acc(0),preint[i].delta_R(1,2)*temp_acc(0),
                        preint[i].delta_R(1,0)*temp_acc(1),preint[i].delta_R(1,1)*temp_acc(1),preint[i].delta_R(1,2)*temp_acc(1),
                        preint[i].delta_R(1,0)*temp_acc(2),preint[i].delta_R(1,1)*temp_acc(2),preint[i].delta_R(1,2)*temp_acc(2);
            Row9 temp_3;
            temp_3 <<   preint[i].delta_R(2,0)*temp_acc(0),preint[i].delta_R(2,1)*temp_acc(0),preint[i].delta_R(2,2)*temp_acc(0),
                        preint[i].delta_R(2,0)*temp_acc(1),preint[i].delta_R(2,1)*temp_acc(1),preint[i].delta_R(2,2)*temp_acc(1),
                        preint[i].delta_R(2,0)*temp_acc(2),preint[i].delta_R(2,1)*temp_acc(2),preint[i].delta_R(2,2)*temp_acc(2);
            d_acc_d_bw[0].row(i) = temp_1*temp_d_R_d_bw;
            d_acc_d_bw[1].row(i) = temp_2*temp_d_R_d_bw;
            d_acc_d_bw[2].row(i) = temp_3*temp_d_R_d_bw;

            temp_acc = preint[i].delta_R * temp_acc;
            Vec3 acc_rot_dt = delta_R_dt_start.transpose()*temp_acc;
            Vec3 d_acc_d_t = (acc_rot_dt - temp_acc)/kNumDtJacobianDelta;
            d_acc_d_dt[0][i] = d_acc_d_t(0);
            d_acc_d_dt[1][i] = d_acc_d_t(1);
            d_acc_d_dt[2][i] = d_acc_d_t(2);
            
            output.col(i) = temp_acc;
        }
        return output;
    }



    inline std::pair<VecX, VecX> linearInterpolation(const VecX& data, const VecX& time, const double var, const SortIndexTracker2<double>& infer_t)
    {
        VecX out_val(infer_t.size());
        VecX out_var(infer_t.size());

        if( time.rows() <2 ){
            throw std::range_error("InterpolateLinear: this function need at least 2 data points to interpolate");
        }

        int ptr = 0;
        double alpha = (data(1) - data(0)) / (time(1) - time(0));
        double beta = data(0) - (alpha*time(0));


        for(size_t i = 0; i < infer_t.size(); ++i){
            if( infer_t.get(i) > time(0))
            {
                bool loop = true;
                while(loop)
                {
                    if(ptr != (time.rows()-2))
                    {
                        if( (infer_t.get(i) <= time(ptr+1)) 
                                &&(infer_t.get(i) > time(ptr)))
                        {
                            loop = false;
                        }
                        else
                        {
                            ptr++;
                            alpha = (data(ptr+1) - data(ptr)) / (time(ptr+1) - time(ptr));
                            beta = data(ptr) - (alpha*time(ptr));
                        }
                    }
                    else
                    {
                        loop = false;
                    }
                }
            }
            out_val(i) = alpha * infer_t.get(i) + beta;
            // Might want more complex policy on variance
            out_var(i) = var;
        }
        return {out_val, out_var};
    }

    inline VecX linearInterpolation(const VecX& data, const VecX& time, const SortIndexTracker2<double>& infer_t)
    {
        auto [val, var] = linearInterpolation(data, time, 0, infer_t);
        return val;
    }


    inline Vec9 perturbationPropagation(const Vec18& eps, const PreintMeas& prev, const PreintMeas& curr)
    {
        Vec3 eps_r1 = eps.segment<3>(0);
        Vec3 eps_v1 = eps.segment<3>(3);
        Vec3 eps_p1 = eps.segment<3>(6);
        Vec3 eps_r2 = eps.segment<3>(9);
        Vec3 eps_v2 = eps.segment<3>(12);
        Vec3 eps_p2 = eps.segment<3>(15);
        
        Vec9 output;
        Mat3 exp_eps_r1 = expMap(eps_r1);
        Mat3 R_exp_eps_r1 = prev.delta_R*exp_eps_r1;
        output.segment<3>(0) = logMap(curr.delta_R.transpose()*exp_eps_r1*curr.delta_R*expMap(eps_r2));
        output.segment<3>(3) = eps_v1 + R_exp_eps_r1*(curr.delta_v + eps_v2);
        output.segment<3>(6) = eps_p1 + R_exp_eps_r1*(curr.delta_p + eps_p2) + prev.dt*eps_v1;
        return output;
    }

    // Numerical propagation, not the most efficient/elegant, but works
    inline Mat9 propagatePreintCov(const PreintMeas& prev, const PreintMeas& curr)
    {
        double quantum = 1e-5;
        MatX d_eps_d_eps(9,18);
        Vec18 eps = Vec18::Zero();
        Vec9 perturbation = perturbationPropagation(eps, prev, curr);
        for(int i = 0; i < 18; ++i)
        {
            eps(i) = quantum;
            d_eps_d_eps.col(i) = (perturbationPropagation(eps, prev, curr) - perturbation)/quantum;
            eps(i) = 0;
        }
        MatX cov = MatX::Zero(18,18);
        cov.block<9,9>(0,0) = prev.cov;
        cov.block<9,9>(9,9) = curr.cov;
        return d_eps_d_eps*cov*d_eps_d_eps.transpose();
    }


    inline Mat3 propagateJacobianRp(const Mat3& R, const Mat3& d_r, const Vec3& p, const Mat3& d_p){

        Mat3 output;
        Mat9_3 d_R = jacobianYX(R)*jacobianExpMapZeroM(d_r);
        output <<
                R.row(0) * d_p + d_R.row(0)*p(0)
                            + d_R.row(3)*p(1)
                            + d_R.row(6)*p(2),
                R.row(1) * d_p + d_R.row(1)*p(0)
                            + d_R.row(4)*p(1)
                            + d_R.row(7)*p(2),
                R.row(2) * d_p + d_R.row(2)*p(0)
                            + d_R.row(5)*p(1)
                            + d_R.row(8)*p(2);

        return output;
    }
    inline Vec3 propagateJacobianRp(const Mat3& R, const Vec3& d_r, const Vec3& p, const Vec3& d_p){

        Vec3 output;
        Vec9 d_R = jacobianYX(R)*jacobianExpMapZeroV(d_r);
        output <<
                R.row(0) * d_p + d_R(0)*p(0)
                               + d_R(3)*p(1)
                               + d_R(6)*p(2),
                R.row(1) * d_p + d_R(1)*p(0)
                               + d_R(4)*p(1)
                               + d_R(7)*p(2),
                R.row(2) * d_p + d_R(2)*p(0)
                               + d_R(5)*p(1)
                               + d_R(8)*p(2);

        return output;
    }

    inline Mat3 propagateJacobianRR(const Mat3& R1, const Mat3& d_r1, const Mat3& R2, const Mat3& d_r2){

        Mat3 output;
        Mat9_3 d_R1 = jacobianYX(R1)*jacobianExpMapZeroM(d_r1);
        Mat9_3 d_R2 = jacobianYX(R2)*jacobianExpMapZeroM(d_r2);
        Mat9_3 d_RR;
        d_RR <<
                R1.row(0) * d_R2.block<3,3>(0,0) + d_R1.row(0)*R2(0,0)
                                                + d_R1.row(3)*R2(1,0)
                                                + d_R1.row(6)*R2(2,0),
                R1.row(1) * d_R2.block<3,3>(0,0) + d_R1.row(1)*R2(0,0)
                                                + d_R1.row(4)*R2(1,0)
                                                + d_R1.row(7)*R2(2,0),
                R1.row(2) * d_R2.block<3,3>(0,0) + d_R1.row(2)*R2(0,0)
                                                + d_R1.row(5)*R2(1,0)
                                                + d_R1.row(8)*R2(2,0),
                R1.row(0) * d_R2.block<3,3>(3,0) + d_R1.row(0)*R2(0,1)
                                                + d_R1.row(3)*R2(1,1)
                                                + d_R1.row(6)*R2(2,1),
                R1.row(1) * d_R2.block<3,3>(3,0) + d_R1.row(1)*R2(0,1)
                                                + d_R1.row(4)*R2(1,1)
                                                + d_R1.row(7)*R2(2,1),
                R1.row(2) * d_R2.block<3,3>(3,0) + d_R1.row(2)*R2(0,1)
                                                + d_R1.row(5)*R2(1,1)
                                                + d_R1.row(8)*R2(2,1),
                R1.row(0) * d_R2.block<3,3>(6,0) + d_R1.row(0)*R2(0,2)
                                                + d_R1.row(3)*R2(1,2)
                                                + d_R1.row(6)*R2(2,2),
                R1.row(1) * d_R2.block<3,3>(6,0) + d_R1.row(1)*R2(0,2)
                                                + d_R1.row(4)*R2(1,2)
                                                + d_R1.row(7)*R2(2,2),
                R1.row(2) * d_R2.block<3,3>(6,0) + d_R1.row(2)*R2(0,2)
                                                + d_R1.row(5)*R2(1,2)
                                                + d_R1.row(8)*R2(2,2);

        output = jacobianLogMap(R1*R2)*d_RR;
        return output;
    }
    inline Vec3 propagateJacobianRR(const Mat3& R1, const Vec3& d_r1, const Mat3& R2, const Vec3& d_r2){

        Vec3 output;
        Vec9 d_R1 = jacobianYX(R1)*jacobianExpMapZeroV(d_r1);
        Vec9 d_R2 = jacobianYX(R2)*jacobianExpMapZeroV(d_r2);
        Vec9 d_RR;
        d_RR <<
                R1.row(0) * d_R2.block<3,1>(0,0) + d_R1(0)*R2(0,0)
                                                 + d_R1(3)*R2(1,0)
                                                 + d_R1(6)*R2(2,0),
                R1.row(1) * d_R2.block<3,1>(0,0) + d_R1(1)*R2(0,0)
                                                 + d_R1(4)*R2(1,0)
                                                 + d_R1(7)*R2(2,0),
                R1.row(2) * d_R2.block<3,1>(0,0) + d_R1(2)*R2(0,0)
                                                 + d_R1(5)*R2(1,0)
                                                 + d_R1(8)*R2(2,0),
                R1.row(0) * d_R2.block<3,1>(3,0) + d_R1(0)*R2(0,1)
                                                 + d_R1(3)*R2(1,1)
                                                 + d_R1(6)*R2(2,1),
                R1.row(1) * d_R2.block<3,1>(3,0) + d_R1(1)*R2(0,1)
                                                 + d_R1(4)*R2(1,1)
                                                 + d_R1(7)*R2(2,1),
                R1.row(2) * d_R2.block<3,1>(3,0) + d_R1(2)*R2(0,1)
                                                 + d_R1(5)*R2(1,1)
                                                 + d_R1(8)*R2(2,1),
                R1.row(0) * d_R2.block<3,1>(6,0) + d_R1(0)*R2(0,2)
                                                 + d_R1(3)*R2(1,2)
                                                 + d_R1(6)*R2(2,2),
                R1.row(1) * d_R2.block<3,1>(6,0) + d_R1(1)*R2(0,2)
                                                 + d_R1(4)*R2(1,2)
                                                 + d_R1(7)*R2(2,2),
                R1.row(2) * d_R2.block<3,1>(6,0) + d_R1(2)*R2(0,2)
                                                 + d_R1(5)*R2(1,2)
                                                 + d_R1(8)*R2(2,2);

        output = jacobianLogMap(R1*R2)*d_RR;
        return output;
    }


    inline PreintMeas combinePreints(const PreintMeas& prev_preint, const PreintMeas& preint)
    {
        if(preint.dt == 0.0)
            return prev_preint;

        PreintMeas temp_preint = preint;
        temp_preint.cov = Mat9::Zero();

        temp_preint.cov = propagatePreintCov(prev_preint, preint);

        // Propagation of acc Jacobians
        temp_preint.d_delta_p_d_bf = prev_preint.d_delta_p_d_bf
                + (temp_preint.dt*prev_preint.d_delta_v_d_bf)
                + (prev_preint.delta_R*temp_preint.d_delta_p_d_bf);

        temp_preint.d_delta_v_d_bf = prev_preint.d_delta_v_d_bf
                + (prev_preint.delta_R * temp_preint.d_delta_v_d_bf);


        // Propagation of gyr Jacobians
        temp_preint.d_delta_p_d_bw = prev_preint.d_delta_p_d_bw
                + (temp_preint.dt*prev_preint.d_delta_v_d_bw)
                + propagateJacobianRp(prev_preint.delta_R, prev_preint.d_delta_R_d_bw, temp_preint.delta_p, temp_preint.d_delta_p_d_bw);

        temp_preint.d_delta_v_d_bw = prev_preint.d_delta_v_d_bw
                + propagateJacobianRp(prev_preint.delta_R, prev_preint.d_delta_R_d_bw, temp_preint.delta_v, temp_preint.d_delta_v_d_bw);
        temp_preint.d_delta_R_d_bw = propagateJacobianRR(temp_preint.delta_R.transpose(), prev_preint.d_delta_R_d_bw, temp_preint.delta_R, temp_preint.d_delta_R_d_bw);

        // Propagation of time-shift Jacobians
        temp_preint.d_delta_p_d_t = prev_preint.d_delta_p_d_t
                + (temp_preint.dt*prev_preint.d_delta_v_d_t)
                + propagateJacobianRp(prev_preint.delta_R, prev_preint.d_delta_R_d_t, temp_preint.delta_p, temp_preint.d_delta_p_d_t);

        temp_preint.d_delta_v_d_t = prev_preint.d_delta_v_d_t
                + propagateJacobianRp(prev_preint.delta_R, prev_preint.d_delta_R_d_t, temp_preint.delta_v, temp_preint.d_delta_v_d_t);
        temp_preint.d_delta_R_d_t = propagateJacobianRR(temp_preint.delta_R.transpose(), prev_preint.d_delta_R_d_t, temp_preint.delta_R, temp_preint.d_delta_R_d_t);


        // Chunck combination
        temp_preint.delta_p = prev_preint.delta_p
                + prev_preint.delta_v*temp_preint.dt
                + prev_preint.delta_R*temp_preint.delta_p;
        temp_preint.delta_v = prev_preint.delta_v
                + prev_preint.delta_R*temp_preint.delta_v;
        temp_preint.delta_R = prev_preint.delta_R*temp_preint.delta_R;

        temp_preint.dt = prev_preint.dt + temp_preint.dt;
        temp_preint.dt_sq_half = 0.5*temp_preint.dt* temp_preint.dt;



        return temp_preint;
    }



}
#endif
