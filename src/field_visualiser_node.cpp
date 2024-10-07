#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include "lice/ros_utils.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "lice/srv/query_dist_field.hpp"


class FieldVisualiser : public rclcpp::Node
{
    public:
        FieldVisualiser() : Node("field_visualiser")
        {
            double range = readFieldDouble(this, "range", 10.0);
            double resolution = readFieldDouble(this, "resolution", 0.05);

            pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("field", 10);

            odom_pose_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/undistortion_pose", 10, std::bind(&FieldVisualiser::callbackOdom, this, std::placeholders::_1));
            map_odom_correction_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/odom_map_correction", 10, std::bind(&FieldVisualiser::callbackCorrection, this, std::placeholders::_1));

            client_ = this->create_client<lice::srv::QueryDistField>("/query_dist_field");


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


            rclcpp::Rate rate(20);

            while(rclcpp::ok())
            {
                rclcpp::spin_some(this->get_node_base_interface());

                if(new_pose_ && pub_->get_subscription_count() > 0)
                {
                    Mat4 map_pose = map_odom_correction_*odom_pose_;
                    Vec3 pos = map_pose.block<3,1>(0,3);


                    auto request = std::make_shared<lice::srv::QueryDistField::Request>();
                    request->dim = 3;
                    request->num_pts = pts_.size();
                    std::vector<Pointd> pts;
                    for(size_t i = 0; i < pts_.size(); i++)
                    {
                        Vec3 pt = pts_[i] + pos;
                        request->pts.push_back(pt[0]);
                        request->pts.push_back(pt[1]);
                        request->pts.push_back(pt[2]);
                        pts.push_back(Pointd(pt,0.0));
                    }


                    while(!client_->wait_for_service(std::chrono::seconds(1)))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                            return;
                        }
                        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
                    }

                    auto result = client_->async_send_request(request);


                    if(rclcpp::spin_until_future_complete(this->get_node_base_interface() , result) != rclcpp::FutureReturnCode::SUCCESS)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Failed to call service");
                        return;
                    }



                    auto response = result.get();

                    for(size_t i = 0; i < pts.size(); i++)
                    {
                        pts[i].i = response->dists[i];
                    }

                    sensor_msgs::msg::PointCloud2 msg = ptsVecToPointCloud2MsgInternal(pts, "map", odom_time_);
                    pub_->publish(msg);

                    new_pose_ = false;
                    RCLCPP_INFO(this->get_logger(), "Published field point cloud");
                }
                rate.sleep();
            }
        }

    private:
        std::vector<Vec3> pts_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr odom_pose_sub_;

        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr map_odom_correction_sub_;


        std::shared_ptr<rclcpp::Client<lice::srv::QueryDistField> > client_;

        bool new_pose_ = false;
        rclcpp::Time odom_time_;
        Mat4 odom_pose_ = Mat4::Identity();
        Mat4 map_odom_correction_ = Mat4::Identity();


        void callbackOdom(const geometry_msgs::msg::TransformStamped::SharedPtr odom_pose)
        {
            odom_time_ = odom_pose->header.stamp;
            odom_pose_ = transformToMat4(odom_pose->transform);
            new_pose_ = true;
            return;
        }

        void callbackCorrection(const geometry_msgs::msg::TransformStamped::SharedPtr map_odom_correction)
        {
            map_odom_correction_ = transformToMat4(map_odom_correction->transform);
            new_pose_ = true;
            return;
        }


        




};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FieldVisualiser>();
    rclcpp::shutdown();
    return 0;
}
