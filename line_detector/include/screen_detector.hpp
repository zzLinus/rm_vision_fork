//
// Created by rhine on 8/3/23.
//

#ifndef BUILD_SCREEN_DETECTOR_HPP
#define BUILD_SCREEN_DETECTOR_HPP

#include <cv_bridge/cv_bridge.h>

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>

using namespace std::chrono_literals;

namespace line_detector {

    class rectangleShape {
    public:
        rectangleShape(const std::vector<cv::Point> &cont, const std::vector<cv::Point> &corn);

        explicit rectangleShape(const std::vector<cv::Point> &corn);

        std::vector<cv::Point> contour;
        std::vector<cv::Point> cornerPoint;
        double area{}, length{};
    };

    class ScreenDetector {
    public:
        explicit ScreenDetector(double init_second);

        bool detectorInitHook(const cv::Mat &img);

        double getRectDis(const rectangleShape &x, const rectangleShape &y);

        void clear();

        cv::Mat drawing_;
        cv::Mat bin_img_;
        cv::Mat gray_img_;
        std::vector<rectangleShape> rect_, rect_list_;

    private:

        void getRectangle();

        double getDistance(const cv::Point& pointO, const cv::Point& pointA);

        double getDist_P2L(const cv::Point& pointP, const cv::Point& pointA, const cv::Point& pointB);

        rectangleShape getMeanRec(const std::vector<cv::Point> &rect);

        void finishInit();

        bool inited_{};

        std::chrono::system_clock::duration init_time_{5};
        std::chrono::system_clock::time_point start_time_{};
        cv::Mat sum_img_;

    };
}

#endif //BUILD_SCREEN_DETECTOR_HPP
