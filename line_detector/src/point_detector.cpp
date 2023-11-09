//
// Created by rhine on 8/3/23.
//

#include "point_detector.hpp"

namespace line_detector {

    PointDetector::PointDetector() {
        clear();
    }

    std::pair<cv::Point2d, cv::Point2d> PointDetector::detectPoint(const cv::Mat &img) {

        drawing_ = img.clone();

        cv::Mat hsv_img;
        cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

        std::vector<cv::Mat> channels, bgr_channels;
        cv::split(img, bgr_channels);
        cv::split(hsv_img, channels);

        double minV, maxV;
        cv::Point minP, maxP;

        auto light = channels[2];

        cv::minMaxLoc(light, &minV, &maxV, &minP, &maxP);

        auto mask = light > std::max((0.6 * maxV), 220.0);
        red_img_ = mask;

        cv::Mat edge;
        cv::Canny(mask, edge, 150, 100, 3);

        auto point_list = getConnected(edge);

        std::cout << "size: " << point_list.size() << std::endl;

        if(point_list.empty()) {
            return {{-1, -1}, {-1, -1}};
        }
        else/* if(point_list.size() == 1) {
            auto mon = cv::moments(point_list[0]);
            cv::Point2d point = {mon.m10 / mon.m00, mon.m01 / mon.m00};
            return {point, point};
        }
        else */{
            std::vector<std::pair<double, cv::Point2d>> ans_list;
            for(unsigned int i = 0; i < point_list.size(); i++) {
                auto mon = cv::moments(point_list[i]);
                cv::Point2d point_center = {mon.m10 / mon.m00, mon.m01 / mon.m00};
                cv::Mat point_mask(mask.size(), CV_8U, cv::Scalar(0));

                auto rect = cv::boundingRect(point_list[i]);
//                rect.width++;
//                rect.height++;
                auto bgr_mean = cv::mean(img(rect));
                ans_list.emplace_back(bgr_mean.val[2] / bgr_mean.val[1], point_center);

//                cv::drawContours(point_mask, point_list, (int)i, 255, -1);
//                auto red = cv::mean(bgr_channels[2], point_mask);
//                auto green = cv::mean(bgr_channels[1], point_mask);
//                ans_list.emplace_back(red.val[0] / green.val[0], point_center);
//                ans_list.emplace_back(1.0, point_center);
            }
            std::sort(ans_list.begin(), ans_list.end(), [](const auto &x, const auto &y) {
                return x.first > y.first;
            });
            auto red_point = ans_list[0].second;
            auto green_point = ans_list[ans_list.size() - 1].second;
            if(ans_list[0].first < 0.9) {
                red_point = {-1, -1};
            }
            if(ans_list[ans_list.size() - 1].first > 1.1) {
                green_point = {-1, -1};
            }
            return{red_point, green_point};
        }
    }

    std::vector<std::vector<cv::Point>> PointDetector::getConnected(const cv::Mat &img) const {

        std::vector<std::vector<cv::Point>> contours, ansContours;

        cv::findContours(img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

        for(const auto &contour : contours) {
            auto area = cv::contourArea(contour);
            if(area > threshold && area < 300) {
                ansContours.push_back(contour);
            }
        }

        return ansContours;
    }

    void PointDetector::clear() {
        inited_ = false;
    }

}
