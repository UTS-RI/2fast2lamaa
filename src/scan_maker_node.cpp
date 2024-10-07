#include "rclcpp/rclcpp.hpp"
#include "lice/ros_utils.h"
#include "lice/utils.h"
#include "lice/types.h"
#include "sensor_msgs/msg/point_cloud2.hpp"




class ScanMakerNode : public rclcpp::Node
{
    public:
        ScanMakerNode()
            : Node("scan_maker")
            , scan_period_(rclcpp::Duration::from_seconds(0.0))
        {

            RCLCPP_INFO(this->get_logger(), "Starting scan_maker node");
            double scan_period = readRequiredFieldDouble(this, "scan_period");
            time_field_multiplier_ = readFieldDouble(this, "point_time_multiplier", 1e-9);

            min_range_ = readFieldDouble(this, "min_range", 0.1);
            max_range_ = readFieldDouble(this, "max_range", 100.0);

            scan_period_ = rclcpp::Duration::from_seconds(scan_period);

            buffer_pts_.reserve(1000000);

            pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_scan", 10);


            sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar_raw_points", 10, std::bind(&ScanMakerNode::pcCallback, this, std::placeholders::_1));
        }

    private:
        rclcpp::Duration scan_period_;
        double time_field_multiplier_;
        bool first_ = true;
        unsigned int counter_ = 0;
        rclcpp::Time first_time_;
        rclcpp::Time next_time_;
        rclcpp::Time current_time_;
        bool has_intensity_ = false;
        bool has_channel_ = false;
        rclcpp::Time previous_pt_time_;

        float min_range_ = 0.1;
        float max_range_ = 100.0;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

        std::vector<Pointf> buffer_pts_;
        std::vector<rclcpp::Time> buffer_times_;


        void pcCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            auto [incoming_pts, temp_has_intensity, temp_has_channel] = pointCloud2MsgToPtsVec<float>(msg, time_field_multiplier_);
            has_intensity_ = temp_has_intensity;
            has_channel_ = temp_has_channel;


            // Sort the points by time
            std::sort(incoming_pts.begin(), incoming_pts.end(), [](const Pointf& a, const Pointf& b) { return a.t < b.t; });


            rclcpp::Time header_time = rclcpp::Time(msg->header.stamp);

            if(first_)
            {
                first_time_ = header_time;
                current_time_ = header_time + rclcpp::Duration::from_seconds(incoming_pts[0].t);
                next_time_ = current_time_ + scan_period_;
                first_ = false;
            }



            int nb_pts = msg->width*msg->height;
            for(int i = 0; i < nb_pts; ++i)
            {
                float range = incoming_pts[i].vec3().norm();
                if(range < min_range_ || range > max_range_)
                {
                    continue;
                }
                rclcpp::Time pt_time = header_time + rclcpp::Duration::from_seconds(incoming_pts[i].t);

                if(pt_time > next_time_)
                {
                    // Sort the buffer of points by time stored in buffer_times
                    std::vector<int> indexes = sortIndexes(buffer_times_);
                    std::vector<Pointf> sorted_buffer_pts;
                    for(size_t i = 0; i < indexes.size(); ++i)
                    {
                        sorted_buffer_pts.push_back(buffer_pts_[indexes[i]]);
                        if( !has_channel_) sorted_buffer_pts.back().channel = kNoChannel;
                        if( i > 0 && (buffer_times_[indexes[i]] - buffer_times_[indexes[i-1]]).seconds() < 0)
                        {
                            RCLCPP_WARN_STREAM(this->get_logger(),"Time between points is negative: " << (buffer_times_[indexes[i]] - buffer_times_[indexes[i-1]]).seconds() << "s");
                        }
                    }

                    sensor_msgs::msg::PointCloud2 pc_msg = ptsVecToPointCloud2MsgInternal(sorted_buffer_pts, "lidar", current_time_);

                    counter_++;
                    pub_->publish(pc_msg);
                    buffer_pts_.clear();
                    buffer_times_.clear();
                    current_time_ = next_time_;
                    next_time_ = current_time_ + scan_period_;
                }

                if(pt_time < current_time_)
                {
                    RCLCPP_WARN_STREAM(this->get_logger(), "Point time is before current time by " << (current_time_ - pt_time).seconds() << "s. Ignoring point.");
                }
                else
                {
                    buffer_pts_.push_back(incoming_pts[i]);
                    buffer_pts_.back().t = (pt_time - current_time_).seconds();
                    buffer_times_.push_back(pt_time);
                }
            }
        }
};





int main(int argc, char **argv)
{


    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanMakerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
