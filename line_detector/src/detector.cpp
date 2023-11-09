// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

// OpenCV
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "detector.hpp"
#include "rm_msgs/msg/detector_msg.hpp"
#include "rm_msgs/msg/debug_msg.hpp"

//using namespace std::chrono_literals;
namespace line_detector {
    Detector::Detector(const rclcpp::NodeOptions &options)
            : Node("line_detector", options) {
        getParams();

        screen_detector_ = std::make_unique<ScreenDetector>(3.0);
        point_detector_ = std::make_unique<PointDetector>();
        detector_task_ = std::make_unique<Task>();

        RCLCPP_INFO(get_logger(), "detector run with %s mode", debug_ ? "debug" : "release");

        start_time_ = std::chrono::system_clock::now();

        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/image_raw", rclcpp::SensorDataQoS(),
                std::bind(&Detector::imageCallback, this, std::placeholders::_1));

        light_point_pub_ = this->create_publisher<rm_msgs::msg::DetectorMsg>("/detector/light_point", 10);

        task_command_sub_ = this->create_subscription<std_msgs::msg::Int32>(
                "serial/task_command", 10,
                std::bind(&Detector::taskCommandCallback, this, std::placeholders::_1));

        if(debug_) {
            gray_img_pub_ = image_transport::create_publisher(this, "/detector/gray_img");
            bin_img_pub_ = image_transport::create_publisher(this, "/detector/bin_img");
            drawing_img_pub_ = image_transport::create_publisher(this, "/detector/drawing_img");
            debugPoint = {300, 300};
            debug_sub_ = this->create_subscription<rm_msgs::msg::DebugMsg>(
                    "/gimbal/debug", 10,
                    std::bind(&Detector::debugCallback, this, std::placeholders::_1));
        }
    }

    void Detector::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg) {
        for (auto end_time = std::chrono::system_clock::now() - 1s; (!fps_.empty()) && (fps_.front() < end_time);) {
            fps_.pop();
        }
        fps_.push(std::chrono::system_clock::now());

        auto img = cv_bridge::toCvShare(img_msg, "bgr8")->image;

//        auto ori_img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
//        std::cout << ori_img.size() << std::endl;
//        cv::Mat img;
//        cv::resize(ori_img, img, {640, 480});
//        std::cout << img.size() << std::endl;

        RCLCPP_INFO(get_logger(), "fps: %ld\n", fps_.size());
        if (!screen_detector_->detectorInitHook(img)) {
            return;
        }

        if(!screen_detector_ -> rect_list_.empty()) {
            if(!inited_) {
                RCLCPP_INFO(get_logger(), "get %zu rect", screen_detector_ -> rect_list_.size());
                detector_task_->setRect(
                        screen_detector_ -> rect_list_[0].cornerPoint,
                        screen_detector_ -> rect_list_[std::min<unsigned int>(1u, screen_detector_ -> rect_list_.size() - 1)].cornerPoint);
                inited_ = true;
                if(!debug_) {
                    detector_task_->setTask(TASK_MODE::TASK_1);
                }
            }
            if(debug_) {
                if(std::chrono::system_clock::now() - start_time_ < 15s) {
                    detector_task_->setTask(TASK_MODE::TASK_1);
                }
                else {
                    detector_task_->setTask(TASK_MODE::TASK_4);
                }
            }
        }
        else {
            screen_detector_->clear();
            RCLCPP_ERROR(get_logger(), "Screen detector init error!");
        }

        auto [red_point, green_point] = point_detector_ -> detectPoint(img);
        auto[light_point, target_point] = detector_task_->updateTask({(int)red_point.x, (int)red_point.y}, {(int)green_point.x, (int)green_point.y});
        rm_msgs::msg::DetectorMsg detector_msg{};
        detector_msg.light_point_x = (int)light_point.x;
        detector_msg.light_point_y = (int)light_point.y;
        detector_msg.target_point_x = (int)target_point.x;
        detector_msg.target_point_y = (int)target_point.y;
        light_point_pub_->publish(detector_msg);

        if(debug_) {
            publishDebug(img, red_point, green_point, img_msg);
        }
//        rclcpp::WallRate loop_rate(30.0);
//        if(rclcpp::ok()) {
//            loop_rate.sleep();
//        }
    }

    void Detector::publishDebug(const cv::Mat &img, const cv::Point2d& red_point, const cv::Point& green_point,
                                const sensor_msgs::msg::Image::ConstSharedPtr &img_msg) {
        auto drawing_img = img.clone();
        std::vector<cv::Scalar> color_used = {{255, 0, 0}, {0, 255, 0},
                                              {0, 0, 255}, {255, 0, 255}};
        for(unsigned int i = 0; i < std::min<uint>(4u, screen_detector_ -> rect_.size()); i++) {
            std::vector<std::vector<cv::Point>> temp_cont = {screen_detector_ -> rect_[i].contour};
            cv::drawContours(drawing_img, temp_cont, -1, color_used[i], 1);
            for(const auto &point : screen_detector_ -> rect_[i].cornerPoint) {
                cv::circle(drawing_img, point, 3, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_AA);
            }
        }
        for(unsigned int i = 0; i < std::min<uint>(3u, screen_detector_ -> rect_list_.size()); i++) {
            std::vector<std::vector<cv::Point>> temp_cont = {screen_detector_ -> rect_list_[i].contour};
            cv::drawContours(drawing_img, temp_cont, -1, {0, 0, 0}, 1);
            for(const auto &point : screen_detector_ -> rect_list_[i].cornerPoint) {
                cv::circle(drawing_img, point, 3, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_AA);
            }
        }
        cv::circle(drawing_img, debugPoint, 3, cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_AA);
        if(red_point.x >= 0) {
            std::cout << "red point at: " << (int)red_point.x << ", " << (int)red_point.y << std::endl;
            cv::circle(drawing_img, {(int)red_point.x, (int)red_point.y}, 3, cv::Scalar(255, 0, 0), cv::FILLED, cv::LINE_AA);
        }
        if(green_point.x >= 0) {
            std::cout << "green point at: " << (int)green_point.x << ", " << (int)green_point.y << std::endl;
            cv::circle(drawing_img, {(int)green_point.x, (int)green_point.y}, 3, cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_AA);
        }
        gray_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "mono8", point_detector_->red_img_).toImageMsg());
        bin_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "mono8", screen_detector_->bin_img_).toImageMsg());
        drawing_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "bgr8", drawing_img).toImageMsg());
    }

    void Detector::getParams() {
        try {
            debug_ = declare_parameter<bool>("debug", true);
        } catch (rclcpp::ParameterTypeException &ex) {
            RCLCPP_ERROR(get_logger(), "The debug provided was invalid");
            throw ex;
        }
    }

    void Detector::debugCallback(const rm_msgs::msg::DebugMsg &msg) {
        debugPoint.x = msg.now_target_point_x;
        debugPoint.y = msg.now_target_point_y;
    }

    void Detector::taskCommandCallback(const std_msgs::msg::Int32 &msg) {
        auto val = msg.data;

        RCLCPP_ERROR(get_logger(), "get task_mode %d", val);
        switch (val) {
            case 0:
                detector_task_->taskContinue();
                break;
            case 1:
                detector_task_->setTask(TASK_MODE::TASK_1);
                break;
            case 2:
                detector_task_->setTask(TASK_MODE::TASK_2);
                break;
            case 3:
                detector_task_->setTask(TASK_MODE::TASK_3);
                break;
            case 4:
                detector_task_->setTask(TASK_MODE::TASK_4);
                break;
            default:
                detector_task_->taskStop();
                break;
        }
    }
}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(line_detector::Detector)
