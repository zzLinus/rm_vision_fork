//
// Created by rhine on 7/16/23.
//

#ifndef BUILD_GIMBAL_CONTROL_HPP
#define BUILD_GIMBAL_CONTROL_HPP

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "pid.hpp"

#include "rm_msgs/msg/serial_send.hpp"
#include "rm_msgs/msg/detector_msg.hpp"
#include "rm_msgs/msg/debug_msg.hpp"

using namespace std::chrono_literals;

namespace gimbal {
    class GimbalControl : public rclcpp::Node {
    public:
        GimbalControl(const rclcpp::NodeOptions &options);

        std::pair<double, double> now_target_point_;

    private:
        void updateGimbal();

        void midPointCallback(const rm_msgs::msg::DetectorMsg &msg);

        pid_config getPidParams(const std::string &pid_name);

        double speed{};

        rclcpp::Publisher<rm_msgs::msg::DebugMsg>::SharedPtr debug_pub_;

        rclcpp::Publisher<rm_msgs::msg::SerialSend>::SharedPtr serial_pub_;

        rclcpp::Subscription<rm_msgs::msg::DetectorMsg>::SharedPtr light_point_sub_;

        rclcpp::TimerBase::SharedPtr gimbal_timer_;

        std::unique_ptr<pid> point_x_pid_, point_y_pid_;
        std::unique_ptr<pid_config> point_pid_config_x_, point_pid_config_y_;

        std::pair<int, int> light_point_, target_point_;
        std::chrono::system_clock::time_point last_msg_time;

        const int base_count_x{1580}, regulate_count_x{220};
        const int base_count_y{1580}, regulate_count_y{220};

        int pwm_count[2];
    };
}

#endif //BUILD_GIMBAL_CONTROL_HPP
