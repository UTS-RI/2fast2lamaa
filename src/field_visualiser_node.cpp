#include "ros/ros.h"
#include <Eigen/Dense>
#include "lice/ros_utils.h"

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include "ffastllamaa/QueryDistField.h"


class FieldVisualiser
{
    public:
        FieldVisualiser()
        {
            ros::NodeHandle nh("~");

            double range = readField<double>(nh, "range", 10.0);
            double resolution = readField<double>(nh, "resolution", 0.05);

            pub_ = nh.advertise<sensor_msgs::PointCloud2>("field", 10);

            odom_pose_sub_ = nh.subscribe("/undistortion_pose", 10, &FieldVisualiser::callbackOdom, this);
            map_odom_correction_sub_ = nh.subscribe("/odom_map_correction", 10, &FieldVisualiser::callbackCorrection, this);

            client_ = nh.serviceClient<ffastllamaa::QueryDistField>("/query_dist_field");

            for(double x = -range; x < range; x += resolution)
            {
                for(double y = -range; y < range; y += resolution)
                {
                    Vec3 pt(x, y, 0.0);
                    if(pt.norm() < range)
                    {
                        pts_.push_back(pt);
                    }
                }
            }

            for(double z = -range; z < range; z += resolution)
            {
                for(double y = -range; y < range; y += resolution)
                {
                    Vec3 pt(0.0, y, z);
                    if(pt.norm() < range)
                    {
                        pts_.push_back(pt);
                    }
                }
            }

            for(double x = -range; x < range; x += resolution)
            {
                for(double z = -range; z < range; z += resolution)
                {
                    Vec3 pt(x, 0.0, z);
                    if(pt.norm() < range)
                    {
                        pts_.push_back(pt);
                    }
                }
            }


            ros::Rate rate(20);

            while(ros::ok())
            {
                ros::spinOnce();

                if(new_pose_ && pub_.getNumSubscribers() > 0)
                {
                    Mat4 map_pose = map_odom_correction_*odom_pose_;
                    Vec3 pos = map_pose.block<3,1>(0,3);


                    ffastllamaa::QueryDistField::Request request;
                    request.dim = 3;
                    request.num_pts = pts_.size();
                    std::vector<Pointd> pts;
                    for(size_t i = 0; i < pts_.size(); i++)
                    {
                        Vec3 pt = pts_[i] + pos;
                        request.pts.push_back(pt[0]);
                        request.pts.push_back(pt[1]);
                        request.pts.push_back(pt[2]);
                        pts.push_back(Pointd(pt,0.0));
                    }

                    ffastllamaa::QueryDistField::Response response;
                    if(!client_.call(request, response))
                    {
                        ROS_ERROR("Failed to call service");
                        return;
                    }

                    for(size_t i = 0; i < pts.size(); i++)
                    {
                        pts[i].i = response.dists[i];
                    }

                    sensor_msgs::PointCloud2 msg = ptsVecToPointCloud2MsgInternal(pts, "map", odom_time_);
                    pub_.publish(msg);

                    new_pose_ = false;
                    ROS_INFO("Published field point cloud");
                }
                rate.sleep();
            }
        }

    private:
        std::vector<Vec3> pts_;

        ros::Publisher pub_;

        ros::Subscriber odom_pose_sub_;

        ros::Subscriber map_odom_correction_sub_;

        ros::ServiceClient client_;

        bool new_pose_ = false;
        ros::Time odom_time_;
        Mat4 odom_pose_ = Mat4::Identity();
        Mat4 map_odom_correction_ = Mat4::Identity();


        void callbackOdom(const geometry_msgs::TransformStampedConstPtr& odom_pose)
        {
            odom_time_ = odom_pose->header.stamp;
            odom_pose_ = transformToMat4(odom_pose->transform);
            new_pose_ = true;
            return;
        }

        void callbackCorrection(const geometry_msgs::TransformStampedConstPtr& map_odom_correction)
        {
            map_odom_correction_ = transformToMat4(map_odom_correction->transform);
            new_pose_ = true;
            return;
        }


        




};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "field_visualiser");
    FieldVisualiser field_visualiser;
    return 0;
}
