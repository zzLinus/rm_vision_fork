//
// Created by rhine on 8/3/23.
//

#ifndef BUILD_POINT_DETECTOR_HPP
#define BUILD_POINT_DETECTOR_HPP

#include "cv_bridge/cv_bridge.h"

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>

namespace line_detector {
    class PointDetector {
    public:
        explicit PointDetector();

        std::pair<cv::Point2d, cv::Point2d> detectPoint(const cv::Mat &img);
        cv::Mat red_img_;
        cv::Mat drawing_;

    private:

        [[nodiscard]] std::vector<std::vector<cv::Point>> getConnected(const cv::Mat &img) const;

        void getColorHsv(const cv::Mat &img);

        void clear();
        bool inited_{};

        cv::Mat origin_img_;

        double param_R = 0.7;
        double param_G = 0.7;
        double threshold = 3;
    };
}

#endif //BUILD_POINT_DETECTOR_HPP
