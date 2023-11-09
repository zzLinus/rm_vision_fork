//
// Created by rhine on 7/5/23.
//

#ifndef CHASSIS_CONTROL_PID_HPP
#define CHASSIS_CONTROL_PID_HPP

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

#include <cstdint>

namespace gimbal {
    enum class PID_MODE{
        PID_POSITION,
        PID_DELTA
    };

    class pid_config {
    public:
        explicit pid_config(double kp, double ki, double kd,
                            double i_max, double out_max, PID_MODE mode);

    protected:
        double kp_{}, ki_{}, kd_{};
        double i_max_{}, out_max_{};
        PID_MODE mode_{};
    };

    class pid : pid_config{
    public:
        explicit pid(double kp, double ki, double kd,
                     double i_max, double out_max, PID_MODE mode);

        explicit pid(const pid_config &config);

        double calc(double ref, double set);

        void clear();

    private:
        void errorDelay(double error_now);

        void bufDelay(double buf_now);

        static void limitOut(double &val, double limit);

        double p_out_{}, i_out_{}, d_out_{}, out_{};
        double error_[3]{}, d_buf_[3]{};
        uint8_t cnt_{};
    };
}

#endif //CHASSIS_CONTROL_PID_HPP