#ifndef DETECTOR_HPP_
#define DETECTOR_HPP_


#include "cv_bridge/cv_bridge.h"

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <vector>
#include <queue>

#include "rm_msgs/msg/detector_msg.hpp"
#include "rm_msgs/msg/debug_msg.hpp"
#include "std_msgs/msg/int32.hpp"
#include "task.hpp"
#include "screen_detector.hpp"
#include "point_detector.hpp"

using namespace std::chrono_literals;

namespace line_detector {
    class Detector : public rclcpp::Node {
    public:
        explicit Detector(const rclcpp::NodeOptions &options);

    private:

        void debugCallback(const rm_msgs::msg::DebugMsg &msg);

        void taskCommandCallback(const std_msgs::msg::Int32 &msg);

        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg);

        void publishDebug(const cv::Mat &img, const cv::Point2d& red_point, const cv::Point& green_point,
                                    const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

        void getParams();

        std::unique_ptr<ScreenDetector> screen_detector_;
        std::unique_ptr<PointDetector> point_detector_;
        std::unique_ptr<Task> detector_task_;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr task_command_sub_;
        rclcpp::Subscription<rm_msgs::msg::DebugMsg>::SharedPtr debug_sub_;
        rclcpp::Publisher<rm_msgs::msg::DetectorMsg>::SharedPtr light_point_pub_;

        image_transport::Publisher result_img_pub_;

        image_transport::Publisher gray_img_pub_;
        image_transport::Publisher bin_img_pub_;
        image_transport::Publisher drawing_img_pub_;

        cv::Point debugPoint;

        bool debug_{};
        bool inited_{};

        cv::Mat hsv_img_;
        cv::Mat left_img_;
        cv::Mat right_img_;
        cv::Mat result_img_;

        cv::Mat origin_img_;

        cv::Mat sum_img_;

        std::queue<std::chrono::system_clock::time_point> fps_;

        std::chrono::system_clock::time_point start_time_;
    };
}  // namespace rm_auto_aim

#endif
