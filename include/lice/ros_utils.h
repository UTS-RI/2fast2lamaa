#pragma once

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Transform.h"

#include "types.h"
#include <Eigen/Dense>

template<class T>
inline void printOption(bool user_defined, std::string field, T value)
{
    std::stringstream stream;
    if(user_defined)
    {
        stream << "[Param] User defined value for " << field << " is " << value;
    }
    else
    {
        stream << "[Param] Default value for " << field << " is " << value;
    }
    ROS_INFO("%s",stream.str().c_str());
}

inline void printOptionError(std::string field)
{
    std::stringstream stream;
    stream << "[Param] It seems that the parameter " << field << " is not provided";
    ROS_ERROR("%s",stream.str().c_str());
    throw std::invalid_argument("Invalid parameter");
}
template<class T>
inline T readField(ros::NodeHandle& n, std::string field, T default_value)
{   
    bool user_defined = false;
    T output;
    if(n.hasParam(field))
    {
        user_defined = true;
        n.getParam(field, output);
    }
    else
    {
        output = default_value;
    }
    printOption(user_defined, field, output);
    return output;
}

template<class T>
inline T readRequiredField(ros::NodeHandle& n, std::string field)
{
    T output;
    if(n.hasParam(field))
    {
        n.getParam(field, output);
    }
    else
    {
        std::stringstream stream;
        stream << "[Param] It seems that the parameter " << field << " is not provided";
        ROS_ERROR("%s",stream.str().c_str());
        throw std::invalid_argument("Invalid parameter");
    }
    printOption(true, field, output);
    return output;
}


enum PointFieldTypes
{
    TIME = 0,
    INTENSITY = 1,
    CHANNEL = 2,
    TYPE = 3,
    SCAN_ID = 4,
    DYNAMIC = 5,
    X = 6,
    Y = 7,
    Z = 8,
    RGB = 9,
    NUM_TYPES = 10
};



inline std::vector<std::pair<int, int>> getPointFields(const std::vector<sensor_msgs::PointField>& fields, bool need_time=false)
{
    std::vector<std::pair<int,int> > output(PointFieldTypes::NUM_TYPES, {-1, -1});
    for(size_t i = 0; i < fields.size(); ++i)
    {
        if((fields[i].name == "time")||(fields[i].name == "point_time_offset")||(fields[i].name == "ts")||(fields[i].name == "t"))
        {
            output[PointFieldTypes::TIME] = {fields[i].offset , fields[i].datatype};
        }
        else if((fields[i].name == "channel")||(fields[i].name == "ring"))
        {
            output[PointFieldTypes::CHANNEL] = {fields[i].offset , fields[i].datatype};
        }
        else if(fields[i].name == "intensity")
        {
            output[PointFieldTypes::INTENSITY] = {fields[i].offset , fields[i].datatype};
        }
        else if(fields[i].name == "type")
        {
            output[PointFieldTypes::TYPE] = {fields[i].offset , fields[i].datatype};
        }
        else if(fields[i].name == "scan_id")
        {
            output[PointFieldTypes::SCAN_ID] = {fields[i].offset , fields[i].datatype};
        }
        else if(fields[i].name == "dynamic")
        {
            output[PointFieldTypes::DYNAMIC] = {fields[i].offset , fields[i].datatype};
        }
        else if(fields[i].name == "x")
        {
            output[PointFieldTypes::X] = {fields[i].offset , fields[i].datatype};
        }
        else if(fields[i].name == "y")
        {
            output[PointFieldTypes::Y] = {fields[i].offset , fields[i].datatype};
        }
        else if(fields[i].name == "z")
        {
            output[PointFieldTypes::Z] = {fields[i].offset , fields[i].datatype};
        }
        else if(fields[i].name == "rgb")
        {
            output[PointFieldTypes::RGB] = {fields[i].offset , fields[i].datatype};
        }
    }
    if(need_time&&(output[PointFieldTypes::TIME].first == -1))
    {
        std::cout << "The point cloud does not seem to contain timestamp information (field 'time', 'ts', or 'point_time_offset'" << std::endl;
    }
    if((output[PointFieldTypes::X].first == -1)||(output[PointFieldTypes::Y].first == -1)||(output[PointFieldTypes::Z].first == -1))
    {
        std::cout << "The point cloud seems to miss at least one component (x, y, or z)" << std::endl;
    }
    return output;
}



template <typename T>
inline void preparePointCloud2Msg(sensor_msgs::PointCloud2& output, const std::vector<PointTemplated<T>>& pts, const std::string& frame_id, const ros::Time& time)
{
    output.header.frame_id = frame_id;
    output.header.stamp = time;
    output.width  = pts.size();
    output.height = 1;
    output.is_bigendian = false;
    output.point_step = 37;
    if(pts.size() == 0)
    {
        return;
    }
    if(pts[0].has_color)
    {
        output.point_step += 4;
    }
    output.row_step = output.point_step * output.width;
    output.fields.resize(9);

    output.fields[0].name = "x";
    output.fields[0].count =1;
    output.fields[0].offset = 0;
    output.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    output.fields[1].name = "y";
    output.fields[1].count =1;
    output.fields[1].offset = 4;
    output.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    output.fields[2].name = "z";
    output.fields[2].count =1;
    output.fields[2].offset = 8;
    output.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    output.fields[3].name = "intensity";
    output.fields[3].count =1;
    output.fields[3].offset = 12;
    output.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    output.fields[4].name = "t";
    output.fields[4].count =1;
    output.fields[4].offset = 16;
    output.fields[4].datatype = sensor_msgs::PointField::FLOAT64;
    output.fields[5].name = "scan_id";
    output.fields[5].count =1;
    output.fields[5].offset = 24;
    output.fields[5].datatype = sensor_msgs::PointField::INT32;
    output.fields[6].name = "channel";
    output.fields[6].count =1;
    output.fields[6].offset = 28;
    output.fields[6].datatype = sensor_msgs::PointField::INT32;
    output.fields[7].name = "type";
    output.fields[7].count =1;
    output.fields[7].offset = 32;
    output.fields[7].datatype = sensor_msgs::PointField::INT32;
    output.fields[8].name = "dynamic";
    output.fields[8].count =1;
    output.fields[8].offset = 36;
    output.fields[8].datatype = sensor_msgs::PointField::UINT8;
    if(pts[0].has_color)
    {
        output.fields.resize(10);
        output.fields[9].name = "rgb";
        output.fields[9].count =1;
        output.fields[9].offset = 37;
        output.fields[9].datatype = sensor_msgs::PointField::FLOAT32;
    }


    output.row_step = output.point_step * output.width;
    output.data.resize(output.point_step*pts.size());
}

inline sensor_msgs::PointCloud2 ptsVecToPointCloud2MsgInternal(const std::vector<Pointd>& pts, const std::string& frame_id, const ros::Time& time)
{
    sensor_msgs::PointCloud2 output;
    preparePointCloud2Msg(output, pts, frame_id, time);
    for(size_t i = 0; i < pts.size(); ++i)
    {
        float x = (float)pts[i].x;
        float y = (float)pts[i].y;
        float z = (float)pts[i].z;
        memcpy(&(output.data[(output.point_step*i) ]), &(x), sizeof(float));
        memcpy(&(output.data[(output.point_step*i) + 4]), &(y), sizeof(float));
        memcpy(&(output.data[(output.point_step*i) + 8]), &(z), sizeof(float));
        memcpy(&(output.data[(output.point_step*i) + 12]), &(pts[i].i), sizeof(float));
        memcpy(&(output.data[(output.point_step*i) + 16]), &(pts[i].t), sizeof(double));
        memcpy(&(output.data[(output.point_step*i) + 24]), &(pts[i].scan_id), sizeof(int));
        memcpy(&(output.data[(output.point_step*i) + 28]), &(pts[i].channel), sizeof(int));
        memcpy(&(output.data[(output.point_step*i) + 32]), &(pts[i].type), sizeof(int));
        output.data[(output.point_step*i) + 36] = pts[i].dynamic;
        if(pts[i].has_color)
        {
            memcpy(&(output.data[(output.point_step*i) + 39]), &(pts[i].r), sizeof(unsigned char));
            memcpy(&(output.data[(output.point_step*i) + 38]), &(pts[i].g), sizeof(unsigned char));
            memcpy(&(output.data[(output.point_step*i) + 37]), &(pts[i].b), sizeof(unsigned char));
        }
    }
    return output;
}

inline sensor_msgs::PointCloud2 ptsVecToPointCloud2MsgInternal(const std::vector<Pointd>& pts, const std_msgs::Header& header)
{
    return ptsVecToPointCloud2MsgInternal(pts, header.frame_id, ros::Time(header.stamp));
}

inline sensor_msgs::PointCloud2 ptsVecToPointCloud2MsgInternal(const std::vector<Pointf>& pts, const std::string& frame_id, const ros::Time& time)
{
    sensor_msgs::PointCloud2 output;
    preparePointCloud2Msg(output, pts, frame_id, time);
    for(size_t i = 0; i < pts.size(); ++i)
    {
        memcpy(&(output.data[(output.point_step*i) ]), &(pts[i].x), sizeof(float));
        memcpy(&(output.data[(output.point_step*i) + 4]), &(pts[i].y), sizeof(float));
        memcpy(&(output.data[(output.point_step*i) + 8]), &(pts[i].z), sizeof(float));
        memcpy(&(output.data[(output.point_step*i) + 12]), &(pts[i].i), sizeof(float));
        memcpy(&(output.data[(output.point_step*i) + 16]), &(pts[i].t), sizeof(double));
        memcpy(&(output.data[(output.point_step*i) + 24]), &(pts[i].scan_id), sizeof(int));
        memcpy(&(output.data[(output.point_step*i) + 28]), &(pts[i].channel), sizeof(int));
        memcpy(&(output.data[(output.point_step*i) + 32]), &(pts[i].type), sizeof(int));
        output.data[(output.point_step*i) + 36] = pts[i].dynamic;
        if(pts[i].has_color)
        {
            memcpy(&(output.data[(output.point_step*i) + 39]), &(pts[i].r), sizeof(unsigned char));
            memcpy(&(output.data[(output.point_step*i) + 38]), &(pts[i].g), sizeof(unsigned char));
            memcpy(&(output.data[(output.point_step*i) + 37]), &(pts[i].b), sizeof(unsigned char));
        }
    }
    return output;
}

inline sensor_msgs::PointCloud2 ptsVecToPointCloud2MsgInternal(const std::vector<Pointf>& pts, const std_msgs::Header& header)
{
    return ptsVecToPointCloud2MsgInternal(pts, header.frame_id, ros::Time(header.stamp));
}



inline std::vector<Pointd> pointCloud2MsgToPtsVecInternal(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::vector<Pointd> output;
    output.resize(msg->width*msg->height);
    bool has_color = (msg->fields.size() > 9);
    for(size_t i=0; i < output.size(); ++i)
    {
        float temp_x, temp_y, temp_z;
        memcpy(&(temp_x), &(msg->data[(msg->point_step*i) + 0]), sizeof(float));
        memcpy(&(temp_y), &(msg->data[(msg->point_step*i) + 4]), sizeof(float));
        memcpy(&(temp_z), &(msg->data[(msg->point_step*i) + 8]), sizeof(float));
        output[i].x = (double)temp_x;
        output[i].y = (double)temp_y;
        output[i].z = (double)temp_z;
        memcpy(&(output[i].i), &(msg->data[(msg->point_step*i) + 12]), sizeof(float));
        memcpy(&(output[i].t), &(msg->data[(msg->point_step*i) + 16]), sizeof(double));
        memcpy(&(output[i].scan_id), &(msg->data[(msg->point_step*i) + 24]), sizeof(int));
        memcpy(&(output[i].channel), &(msg->data[(msg->point_step*i) + 28]), sizeof(int));
        memcpy(&(output[i].type), &(msg->data[(msg->point_step*i) + 32]), sizeof(int));
        output[i].dynamic = msg->data[(msg->point_step*i) + 36];
        if(has_color)
        {
            output[i].r = msg->data[(msg->point_step*i) + 39];
            output[i].g = msg->data[(msg->point_step*i) + 38];
            output[i].b = msg->data[(msg->point_step*i) + 37];
            output[i].has_color = true;
        }

    }
    return output;
}

inline std::vector<Pointf> pointCloud2MsgToPtsVecInternalFloat(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::vector<Pointf> output;
    output.resize(msg->width*msg->height);
    bool has_color = (msg->fields.size() > 9);
    for(size_t i=0; i < output.size(); ++i)
    {
        memcpy(&(output[i].x), &(msg->data[(msg->point_step*i) + 0]), sizeof(float));
        memcpy(&(output[i].y), &(msg->data[(msg->point_step*i) + 4]), sizeof(float));
        memcpy(&(output[i].z), &(msg->data[(msg->point_step*i) + 8]), sizeof(float));
        memcpy(&(output[i].i), &(msg->data[(msg->point_step*i) + 12]), sizeof(float));
        memcpy(&(output[i].t), &(msg->data[(msg->point_step*i) + 16]), sizeof(double));
        memcpy(&(output[i].scan_id), &(msg->data[(msg->point_step*i) + 24]), sizeof(int));
        memcpy(&(output[i].channel), &(msg->data[(msg->point_step*i) + 28]), sizeof(int));
        memcpy(&(output[i].type), &(msg->data[(msg->point_step*i) + 32]), sizeof(int));
        output[i].dynamic = msg->data[(msg->point_step*i) + 36];
        if(has_color)
        {
            output[i].r = msg->data[(msg->point_step*i) + 39];
            output[i].g = msg->data[(msg->point_step*i) + 38];
            output[i].b = msg->data[(msg->point_step*i) + 37];
            output[i].has_color = true;
        }
    }
    return output;
}

// Function to read a PointCloud2 message and convert it to a vector of points
template <typename T>
inline std::tuple<std::vector<PointTemplated<T> >, bool, bool> pointCloud2MsgToPtsVec(const sensor_msgs::PointCloud2ConstPtr& msg, const double time_scale = 1e-9, bool need_time = true)
{
    std::vector<PointTemplated<T>> output;
    output.resize(msg->width*msg->height);
    std::vector<std::pair<int,int> > fields = getPointFields(msg->fields, need_time);
    bool has_intensity = (fields[PointFieldTypes::INTENSITY].first != -1);
    bool has_channel = (fields[PointFieldTypes::CHANNEL].first != -1);
    bool has_type = (fields[PointFieldTypes::TYPE].first != -1);
    bool has_scan_id = (fields[PointFieldTypes::SCAN_ID].first != -1);
    bool has_dynamic = (fields[PointFieldTypes::DYNAMIC].first != -1);
    bool has_time = (fields[PointFieldTypes::TIME].first != -1);
    bool has_color = (fields[PointFieldTypes::RGB].first != -1);
    for(size_t i = 0; i < output.size(); ++i)
    {
        float temp_x, temp_y, temp_z;
        memcpy(&(temp_x), &(msg->data[(msg->point_step*i) + fields[PointFieldTypes::X].first]), sizeof(float));
        memcpy(&(temp_y), &(msg->data[(msg->point_step*i) + fields[PointFieldTypes::Y].first]), sizeof(float));
        memcpy(&(temp_z), &(msg->data[(msg->point_step*i) + fields[PointFieldTypes::Z].first]), sizeof(float));
        output[i].x = (T)temp_x;
        output[i].y = (T)temp_y;
        output[i].z = (T)temp_z;
        if(has_time)
        {
            if(fields[PointFieldTypes::TIME].second == sensor_msgs::PointField::FLOAT64)
            {
                memcpy(&(output[i].t), &(msg->data[(msg->point_step*i) + fields[PointFieldTypes::TIME].first]), sizeof(double));
            }
            else if(fields[PointFieldTypes::TIME].second == sensor_msgs::PointField::FLOAT32)
            {
                float temp_t;
                memcpy(&(temp_t), &(msg->data[(msg->point_step*i) + fields[PointFieldTypes::TIME].first]), sizeof(float));
                output[i].t = (double)temp_t;
            }
            else if(fields[PointFieldTypes::TIME].second == sensor_msgs::PointField::UINT32)
            {
                uint32_t temp_t;
                memcpy(&(temp_t), &(msg->data[(msg->point_step*i) + fields[PointFieldTypes::TIME].first]), sizeof(uint32_t));
                output[i].t = ((double)temp_t) * time_scale;
            }
            else
            {
                std::cout << "The time field is not of type float32 or float64 or unit32" << std::endl;
            }
        }
        else
        {
            output[i].t = msg->header.stamp.toSec();
        }
        if(has_intensity)
        {
            memcpy(&(output[i].i),&(msg->data[(msg->point_step*i) + fields[PointFieldTypes::INTENSITY].first]), sizeof(float));
        }
        if(has_type)
        {
            memcpy(&(output[i].type),&(msg->data[(msg->point_step*i) + fields[PointFieldTypes::TYPE].first]), sizeof(int));
        }
        if(has_channel)
        {
            if(fields[PointFieldTypes::CHANNEL].second == sensor_msgs::PointField::UINT16)
            {
                uint16_t temp_channel;
                memcpy(&(temp_channel),&(msg->data[(msg->point_step*i) + fields[PointFieldTypes::CHANNEL].first]), sizeof(uint16_t));
                output[i].channel = (int)temp_channel;
            }
            else if(fields[PointFieldTypes::CHANNEL].second == sensor_msgs::PointField::INT32)
            {
                int32_t temp_channel;
                memcpy(&(temp_channel),&(msg->data[(msg->point_step*i) + fields[PointFieldTypes::CHANNEL].first]), sizeof(int32_t));
                output[i].channel = (int)temp_channel;
            }
            else if(fields[PointFieldTypes::CHANNEL].second == sensor_msgs::PointField::UINT32)
            {
                uint32_t temp_channel;
                memcpy(&(temp_channel),&(msg->data[(msg->point_step*i) + fields[PointFieldTypes::CHANNEL].first]), sizeof(uint32_t));
                output[i].channel = (int)temp_channel;
            }
            else if(fields[PointFieldTypes::CHANNEL].second == sensor_msgs::PointField::INT16)
            {
                int16_t temp_channel;
                memcpy(&(temp_channel),&(msg->data[(msg->point_step*i) + fields[PointFieldTypes::CHANNEL].first]), sizeof(int16_t));
                output[i].channel = (int)temp_channel;
            }
            else if(fields[PointFieldTypes::CHANNEL].second == sensor_msgs::PointField::INT8)
            {
                int8_t temp_channel;
                memcpy(&(temp_channel),&(msg->data[(msg->point_step*i) + fields[PointFieldTypes::CHANNEL].first]), sizeof(int8_t));
                output[i].channel = (int)temp_channel;
            }
            else if(fields[PointFieldTypes::CHANNEL].second == sensor_msgs::PointField::UINT8)
            {
                uint8_t temp_channel;
                memcpy(&(temp_channel),&(msg->data[(msg->point_step*i) + fields[PointFieldTypes::CHANNEL].first]), sizeof(uint8_t));
                output[i].channel = (int)temp_channel;
            }
            else
            {
                std::cout << "The channel field is of unknown type" << std::endl;
            }
        }
        if(has_scan_id)
        {
            memcpy(&(output[i].scan_id),&(msg->data[(msg->point_step*i) + fields[PointFieldTypes::SCAN_ID].first]), sizeof(int));
        }
        if(has_dynamic)
        {
            output[i].dynamic = msg->data[(msg->point_step*i) + fields[PointFieldTypes::DYNAMIC].first];
        }
        if(has_color)
        {
            memcpy(&(output[i].r),&(msg->data[(msg->point_step*i) + fields[PointFieldTypes::RGB].first + 2]), sizeof(uint8_t));
            memcpy(&(output[i].g),&(msg->data[(msg->point_step*i) + fields[PointFieldTypes::RGB].first + 1]), sizeof(uint8_t));
            memcpy(&(output[i].b),&(msg->data[(msg->point_step*i) + fields[PointFieldTypes::RGB].first + 0]), sizeof(uint8_t));
            output[i].has_color = true;
        }
    }
    return {output, has_intensity, has_channel};
}


inline Mat4 transformToMat4(const geometry_msgs::Transform& msg)
{
    Mat4 output = Mat4::Identity();
    output(0,3) = msg.translation.x;
    output(1,3) = msg.translation.y;
    output(2,3) = msg.translation.z;
    Eigen::Quaterniond q(msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z);
    output.block<3,3>(0,0) = q.toRotationMatrix();
    return output;
}

inline geometry_msgs::Transform mat4ToTransform(const Mat4& mat)
{
    geometry_msgs::Transform output;
    output.translation.x = mat(0,3);
    output.translation.y = mat(1,3);
    output.translation.z = mat(2,3);
    Eigen::Quaterniond q(mat.block<3,3>(0,0));
    output.rotation.x = q.x();
    output.rotation.y = q.y();
    output.rotation.z = q.z();
    output.rotation.w = q.w();
    return output;
}
