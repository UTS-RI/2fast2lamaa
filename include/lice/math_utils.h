#pragma once


#include "lice/types.h"
#include <random>

const double kExpNormTolerance = 1e-14;


inline Mat3 toSkewSymMat(const Vec3& rot_vec)
{
    Mat3 skew_mat;
    skew_mat << 0.0, -rot_vec(2), rot_vec(1),
                rot_vec(2), 0.0, -rot_vec(0),
                -rot_vec(1), rot_vec(0), 0.0;
    return skew_mat;

}

inline Mat3_4 jacobianQuatRotation(const Vec4& q, const Vec3& x)
{
    Mat3_4 output;
    const double& q1 = q[0];
    const double& q2 = q[1];
    const double& q3 = q[2];
    const double& q4 = q[3];
    const double& x1 = x[0];
    const double& x2 = x[1];
    const double& x3 = x[2];
    output << 
        2*q3*x3 - 2*q4*x2,           2*q3*x2 + 2*q4*x3, 2*q1*x3 + 2*q2*x2 - 4*q3*x1, 2*q2*x3 - 2*q1*x2 - 4*q4*x1,
        2*q4*x1 - 2*q2*x3, 2*q3*x1 - 4*q2*x2 - 2*q1*x3,           2*q2*x1 + 2*q4*x3, 2*q1*x1 + 2*q3*x3 - 4*q4*x2,
        2*q2*x2 - 2*q3*x1, 2*q1*x2 - 4*q2*x3 + 2*q4*x1, 2*q4*x2 - 4*q3*x3 - 2*q1*x1,           2*q2*x1 + 2*q3*x2;

    return output;
}


// Righthand Jacobian of SO3 Exp mapping
template<typename T>
inline Eigen::Matrix<T, 3, 3> jacobianRighthandSO3(const Eigen::Matrix<T, 3, 1> rot_vec)
{    
    Eigen::Matrix<T, 3, 3> output = Eigen::Matrix<T, 3, 3>::Identity();
    T vec_norm = rot_vec.norm();

            
    Eigen::Matrix<T, 3, 1> vec = rot_vec;

    if(vec_norm > kExpNormTolerance)
    {    

        Eigen::Matrix<T, 3, 3> skew_mat;
        skew_mat << T(0.0), T(-vec(2)), T(vec(1)),
                    T(vec(2)), T(0.0), T(-vec(0)),
                    T(-vec(1)), T(vec(0)), T(0.0);
                
        output += ( (vec_norm - sin(vec_norm)) / (vec_norm*vec_norm*vec_norm) )*skew_mat*skew_mat  - ( (1.0 - cos(vec_norm))/(vec_norm*vec_norm) )*skew_mat;
    }    
    return output;
}

// Lefthand Jacobian of SO3 Exp mapping
inline Mat3 jacobianLefthandSO3( Vec3 rot_vec)
{
    return jacobianRighthandSO3<double>(-rot_vec);
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

inline std::pair<Vec3, Vec3> invertTransform(const Vec3& p, const Vec3& rot)
{
    // Convert to rotation matrix
    Mat3 R = expMap(rot);
    // Invert rotation matrix
    Mat3 R_inv = R.transpose();
    // Invert translation
    Vec3 p_inv = -R_inv*p;
    // Convert back to rotation vector
    Vec3 rot_inv = logMap(R_inv);
    return {p_inv, rot_inv};
}


inline std::pair<Vec3, Vec3> combineTransforms(const Vec3& p0, const Vec3& rot0, const Vec3& p1, const Vec3& rot1)
{
    // Convert to rotation matrix
    Mat3 R0 = expMap(rot0);
    Mat3 R1 = expMap(rot1);
    // Combine rotation matrix
    Mat3 R = R0*R1;
    // Combine translation
    Vec3 p = R0*p1 + p0;
    // Convert back to rotation vector
    Vec3 rot = logMap(R);
    return {p, rot};
}

inline std::pair<double, double> distanceBetweenTransforms(const Mat4& T0, const Mat4& T1)
{
    Mat4 T = T0.inverse()*T1;
    Vec3 p = T.block<3, 1>(0, 3);
    Vec3 rot = logMap(T.block<3, 3>(0, 0));
    return {p.norm(), rot.norm()};
}


inline Mat4 posRotToTransform(const Vec3& pos, const Vec3& rot)
{
    Mat4 T = Mat4::Identity();
    T.block<3, 3>(0, 0) = expMap(rot);
    T.block<3, 1>(0, 3) = pos;
    return T;
}

// Cartesian to polar coordinates (r, elevation, azimuth)
inline Vec3 toPolar(const Vec3& xyz)
{
    double xy2 = (xyz[0]*xyz[0]) + (xyz[1]*xyz[1]);
    return Vec3(
        std::sqrt(xy2 + (xyz[2]*xyz[2])),
        std::atan2(std::sqrt(xy2), xyz[2]),
        std::atan2(xyz[1], xyz[0]));
}

inline Vec3 toXYZ(const Vec3& pol)
{
    double s = std::sin(pol[1]);
    return Vec3(
        pol[0]*s*std::cos(pol[2]),
        pol[0]*s*std::sin(pol[2]),
        pol[0]*std::cos(pol[1]));
}
