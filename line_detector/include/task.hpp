//
// Created by rhine on 8/4/23.
//

#ifndef BUILD_TASK_HPP
#define BUILD_TASK_HPP

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace line_detector {

    enum class TASK_MODE {
        TASK_STOP,
        TASK_1,
        TASK_2,
        TASK_3,
        TASK_4,
    };

    class Task {
    public:
        explicit Task();

        void setRect(const std::vector<cv::Point> &rect1, const std::vector<cv::Point> &rect2);

        void setTask(TASK_MODE task_mode);

        void taskStop();

        void taskContinue();

        std::pair<cv::Point, cv::Point> updateTask(const cv::Point& red_point, const cv::Point& green_point);

    private:

        TASK_MODE task_mode_, last_task_;

        std::vector<cv::Point> big_rect_, small_rect_;
        std::vector<cv::Point> big_rect_contour_, small_rect_contour_;

        std::chrono::system_clock::time_point start_time_;
        std::chrono::system_clock::duration last_task_time_;
    };
}

#endif //BUILD_TASK_HPP
