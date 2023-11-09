//
// Created by rhine on 8/3/23.
//
#include "screen_detector.hpp"

namespace line_detector {

    ScreenDetector::ScreenDetector(double init_second) {
        init_time_ = (long long)(init_second * 1000) * 1ms;
        clear();
    }

    bool ScreenDetector::detectorInitHook(const cv::Mat &img) {
        drawing_ = img.clone();
        if(inited_) {
            return true;
        }
        if(sum_img_.size != img.size) {
            sum_img_ = cv::Mat(img.size(), CV_32F, cv::Scalar(0));
        }
        cv::cvtColor(img, gray_img_, cv::COLOR_RGB2GRAY);

        cv::adaptiveThreshold(gray_img_, bin_img_, 255,
                              cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 11, 1);
        sum_img_ += 0.01 * bin_img_;

        if(std::chrono::system_clock::now() - start_time_ < init_time_) {
            return false;
        }
        else {
            std::cout << "finished!\n";
            finishInit();
            inited_ = true;
            return true;
        }
    }

    void ScreenDetector::finishInit() {
        double minV, maxV;
        cv::Point minP, maxP;

        cv::minMaxLoc(sum_img_, &minV, &maxV, &minP, &maxP);
        sum_img_ = (sum_img_ - minV) / (maxV - minV) * 255;
//        float minVal = 1e9f, maxVal = 0.0f;
//        for(int i = 0; i < sum_img_.size[0]; i++) {
//            for(int j = 0; j < sum_img_.size[1]; j++) {
//                if(sum_img_.at<float>(i, j) < minVal) {
//                    minVal = sum_img_.at<float>(i, j);
//                }
//                else if(sum_img_.at<float>(i, j) > maxVal) {
//                    maxVal = sum_img_.at<float>(i, j);
//                }
//            }
//        }
        bin_img_ = cv::Mat(sum_img_.size(), CV_8U);
        for(int i = 0; i < sum_img_.size[0]; i++) {
            for(int j = 0; j < sum_img_.size[1]; j++) {
                bin_img_.at<uchar>(i, j) = (uchar)sum_img_.at<float>(i, j);
            }
        }
        cv::threshold(bin_img_, bin_img_, 60, 255, cv::THRESH_BINARY);
        cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::erode(bin_img_, bin_img_, element);
//        cv::dilate(bin_img_, bin_img_, element);
        getRectangle();
        std::sort(rect_.begin(), rect_.end(), [](const auto &x, const auto &y) {
            return x.area > y.area;
        });

        rect_list_.clear();

        for(unsigned int i = 0; i < rect_.size(); i++) {
            unsigned int step = 0;
            std::vector<rectangleShape> sim_rec;
            for(unsigned int j = i + 1; j < rect_.size(); j++) {
                if(getRectDis(rect_[i], rect_[j]) < 200) {
                    sim_rec.push_back(rect_[j]);
                    step = j;
                }
            }
            if(sim_rec.empty()) {
                rect_list_.push_back(rect_[i]);
            }
            else {
                std::vector<cv::Point> cornerPoint = rect_[i].cornerPoint;
                for(const auto &sim_t : sim_rec) {
                    for(int k = 0; k < 4; k++) {
                        double min_dis = 1e9;
                        cv::Point min_point = sim_t.cornerPoint[0];
                        for(const auto &point : sim_t.cornerPoint) {
                            auto dis = getDistance(rect_[i].cornerPoint[k], point);
                            if(dis < min_dis) {
                                min_dis = dis;
                                min_point = point;
                            }
                        }
                        cornerPoint[k] += min_point;
                    }
                }
                for(auto &point : cornerPoint) {
                    point.x = point.x / (sim_rec.size() + 1);
                    point.y = point.y / (sim_rec.size() + 1);
                }
                rect_list_.emplace_back(cornerPoint);
            }
            i = step ? : i;
        }
        std::sort(rect_list_.begin(), rect_list_.end(), [](const auto &x, const auto &y) {
            return x.area > y.area;
        });
    }

    void ScreenDetector::getRectangle() {
        std::vector<std::vector<cv::Point>> contours, rectContours;

        cv::findContours(bin_img_, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

        for (const auto &cont: contours) {
            auto length = cv::arcLength(cont, true);
            std::vector<cv::Point> hull;
            cv::convexHull(cv::Mat(cont), hull, false);
            auto areaContour = cv::contourArea(cont);
            auto areaHull = cv::contourArea(hull);
            if (areaHull > 10000 && length < 2000) {
                auto rectangle = areaContour / areaHull;
                auto circularity = (4.0 * CV_PI * areaContour) / (length * length);
                if (rectangle > 0.8 && circularity < 0.9) {
                    rectContours.push_back(hull);
                }
            }
        }
        cv::drawContours(drawing_, rectContours, -1, {255, 0, 0}, 1);

        for (auto &rect: rectContours) {
            std::vector<std::pair<cv::Point, unsigned int>> connorPoint(3);
            std::vector<cv::Point> connor4_add(3);  //先找到的三个角点
            for (unsigned int point_index = 0; point_index < connorPoint.size(); point_index++) {
                double maxDistance = 0;
                for (unsigned int rec_index = 0; rec_index < rect.size(); rec_index++) {
                    double distance = 0;
                    if (point_index == 0) {
                        distance = getDistance(rect[rec_index], rect[0]);
                    } else if (point_index == 1) {
                        distance = getDistance(rect[rec_index], connorPoint[0].first);
                    } else if (point_index == 2) {
                        distance = getDistance(rect[rec_index], connorPoint[0].first) +
                                   getDistance(rect[rec_index], connorPoint[1].first);
                    }
                    if (distance > maxDistance) {
                        maxDistance = distance;
                        connorPoint[point_index].first = rect[rec_index];
                        connorPoint[point_index].second = rec_index;
                    }
                }
            }

            std::sort(connorPoint.begin(), connorPoint.end(), [](const auto &x, const auto &y) {
                return x.second != y.second && x.second < y.second;
            });

            auto getId = [&connorPoint](unsigned int x) {
                if (x > connorPoint[1].second && x < connorPoint[2].second) {
                    return 0;
                } else if (x > connorPoint[0].second && x < connorPoint[1].second) {
                    return 1;
                } else if (x < connorPoint[0].second || x > connorPoint[2].second) {
                    return 2;
                } else {
                    return -1;
                }
            };

            const std::vector<std::vector<int>> map_id = {{1, 2}, {0, 1}, {0, 2}};

            std::vector<double> maxDistance(3, 0);
            for (unsigned int rec_index = 0; rec_index < rect.size(); rec_index++) {
                auto id = getId(rec_index);
                if (id == -1) {
                    continue;
                }
                auto &recPoint = rect[rec_index];

                std::vector<double> dis_list = {
                        getDistance(recPoint, connorPoint[0].first),
                        getDistance(recPoint, connorPoint[1].first),
                        getDistance(recPoint, connorPoint[2].first)
                };
                double d = dis_list[map_id[id][0]] + dis_list[map_id[id][1]];
                if (d > maxDistance[id]) {
                    maxDistance[id] = d;
                    connor4_add[id] = recPoint;
                }
            }

            auto uint_d = std::min(getDistance(connorPoint[1].first, connorPoint[2].first),
                                   std::min(getDistance(connorPoint[0].first, connorPoint[1].first),
                                            getDistance(connorPoint[0].first, connorPoint[2].first)));

            for(int id = 0; id < 3; id++) {
                if(getDist_P2L(connor4_add[id], connorPoint[map_id[id][0]].first, connorPoint[map_id[id][1]].first) > 50
                && getDistance(connor4_add[id], connorPoint[map_id[id][0]].first) > 0.8 * uint_d
                && getDistance(connor4_add[id], connorPoint[map_id[id][1]].first) > 0.8 * uint_d) {
                    connorPoint.emplace_back(connor4_add[id], 0);
                    auto temp_connor = {connorPoint[0].first, connorPoint[1].first, connorPoint[2].first, connorPoint[3].first};
                    rect_.emplace_back(rect, temp_connor);
                    break;
                }
            }
        }
    }

    double ScreenDetector::getDistance(const cv::Point &pointO, const cv::Point &pointA) {
        double distance = pow((pointO.x - pointA.x), 2) + pow((pointO.y - pointA.y), 2);
        distance = sqrt(distance);

        return distance;
    }

    double ScreenDetector::getDist_P2L(const cv::Point &pointP, const cv::Point &pointA, const cv::Point &pointB) {
        int A = pointA.y - pointB.y;
        int B = pointB.x - pointA.x;
        int C = pointA.x * pointB.y - pointA.y * pointB.x;
        //代入点到直线距离公式
        double distance = ((double) abs(A * pointP.x + B * pointP.y + C)) / sqrt(A * A + B * B);
        return distance;
    }

    void ScreenDetector::clear() {
        inited_ = false;
        sum_img_ = cv::Mat(0, 0, CV_32F);
        start_time_ = std::chrono::system_clock::now();
        rect_.clear();
        rect_list_.clear();
    }

    double ScreenDetector::getRectDis(const rectangleShape &x, const rectangleShape &y) {
        double dis = 0;
        for(const auto &point_x : x.cornerPoint) {
            double min_dis = 1e9;
            for(const auto &point_y : y.cornerPoint) {
                min_dis = std::min(min_dis, getDistance(point_x, point_y));
            }
            dis += min_dis;
        }
        return dis;
    }

    rectangleShape::rectangleShape(const std::vector<cv::Point> &cont, const std::vector<cv::Point> &corn) :
    contour(cont), cornerPoint(corn) {
        length = cv::arcLength(contour, true);
        area = cv::contourArea(contour, true);
    }

    rectangleShape::rectangleShape(const std::vector<cv::Point> &corn) : cornerPoint(corn) {
        cv::convexHull(corn, contour);
        length = cv::arcLength(contour, true);
        area = cv::contourArea(contour, true);
    }
}
