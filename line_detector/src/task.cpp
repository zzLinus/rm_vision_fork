//
// Created by rhine on 8/4/23.
//

#include <cv_bridge/cv_bridge.h>
#include "task.hpp"

namespace line_detector {

    Task::Task() {
        start_time_ = std::chrono::system_clock::now() - 300ms;
        task_mode_ = TASK_MODE::TASK_STOP;
        last_task_ = TASK_MODE::TASK_STOP;
        last_task_time_ = 0s;
    }

    void Task::setRect(const std::vector<cv::Point> &rect1, const std::vector<cv::Point> &rect2) {
        big_rect_ = rect1;
        small_rect_ = rect2;
        cv::convexHull(rect1, big_rect_contour_);
        cv::convexHull(rect2, small_rect_contour_);
    }

    void Task::setTask(TASK_MODE task_mode) {
        if(task_mode != task_mode_) {
            start_time_ = std::chrono::system_clock::now();
            task_mode_ = task_mode;
        }
    }

    void Task::taskStop() {
        if(task_mode_ != TASK_MODE::TASK_STOP) {
            task_mode_ = TASK_MODE::TASK_STOP;
            last_task_time_ = std::chrono::system_clock::now()  - start_time_;
        }
    }

    void Task::taskContinue() {
        if(task_mode_ == TASK_MODE::TASK_STOP) {
            task_mode_ = last_task_;
            start_time_ = std::chrono::system_clock::now() - last_task_time_;
            last_task_time_ = 0s;
        }
    }

    std::pair<cv::Point, cv::Point> Task::updateTask(const cv::Point& red_point, const cv::Point& green_point) {
        if(task_mode_ == TASK_MODE::TASK_STOP) {
            return {{-2, -2}, {-2, -2}};
        }
        else if(task_mode_ == TASK_MODE::TASK_1) {
            auto mon = cv::moments(big_rect_contour_);
            return {red_point, {(int)(mon.m10 / mon.m00), (int)(mon.m01 / mon.m00)}};
        }
        else if(task_mode_ == TASK_MODE::TASK_2) {
            auto now_time = std::chrono::system_clock::now();
            auto period = 5s;
            for(int i = 0; i <= 4; i++) {
                if(now_time - start_time_ < (i + 1) * period) {
                    return {red_point, big_rect_contour_[(i & 3)]};
                }
            }
            setTask(TASK_MODE::TASK_STOP);
            return {{-1, -1}, {-1, -1}};
        }
        else if(task_mode_ == TASK_MODE::TASK_3) {
            auto now_time = std::chrono::system_clock::now();
            auto period = 5s;
            for(int i = 0; i <= 4; i++) {
                if(now_time - start_time_ < (i + 1) * period) {
                    return {red_point, small_rect_contour_[i & 3]};
                }
            }
            setTask(TASK_MODE::TASK_STOP);
            return {{-1, -1}, {-1, -1}};
        }
        else if(task_mode_ == TASK_MODE::TASK_4) {
            return {green_point, red_point};
        }
        return {{-1, -1}, {-1, -1}};
    }
}