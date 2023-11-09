//
// Created by rhine on 7/5/23.
//

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

//C++ System

#include <cstdint>

#include "pid.hpp"

#include "iostream"

namespace gimbal {

    pid_config::pid_config(double kp, double ki, double kd, double i_max, double out_max, PID_MODE mode) :
    kp_(kp), ki_(ki), kd_(kd), i_max_(i_max), out_max_(out_max), mode_(mode) {}

    pid::pid(const double kp, const double ki, const double kd,
             const double i_max, const double out_max, PID_MODE mode) :
             pid_config(kp, ki, kd, i_max, out_max, mode) {
        clear();
    }

    pid::pid(const pid_config &config) : pid_config(config) {
        clear();
    }

    double pid::calc(const double ref, const double set) {
        errorDelay(set - ref);
        if(mode_ == PID_MODE::PID_POSITION) {
            p_out_ = kp_ * error_[0];
            i_out_ += ki_ * error_[0];
            if(cnt_ > 0){
                bufDelay(error_[0] - error_[1]);
            } else {
                bufDelay(0);
            }
            d_out_ = kd_ * d_buf_[0];
            limitOut(i_out_, i_max_);
            out_ = p_out_ + i_out_ + d_out_;
            limitOut(out_, out_max_);
        } else if(mode_ == PID_MODE::PID_DELTA) {
            p_out_ = kp_ * (error_[0] - error_[1]);
            i_out_ = ki_ * error_[0];
            if(cnt_ > 1){
                bufDelay(error_[0] - 2.0 * error_[1] + error_[2]);
            } else if(cnt_ > 0) {
                bufDelay(error_[0] - error_[1]);
                cnt_ += 1;
            } else {
                bufDelay(0);
                cnt_ += 1;
            }
            d_out_ = kd_ * d_buf_[0];
            out_ += p_out_ + i_out_ + d_out_;
            limitOut(out_, out_max_);
        }
        return out_;
    }

    void pid::clear() {
        for(auto & x : error_) {
            x = 0;
        }
        for(auto & x : d_buf_) {
            x = 0;
        }
        p_out_ = 0;
        i_out_ = 0;
        d_out_ = 0;
        cnt_ = 0;
        if(mode_ != PID_MODE::PID_DELTA) {
            out_ = 0;
        }
    }

    void pid::errorDelay(const double error_now) {
        error_[2] = error_[1];
        error_[1] = error_[0];
        error_[0] = error_now;
    }

    void pid::bufDelay(const double buf_now) {
        d_buf_[2] = d_buf_[1];
        d_buf_[1] = d_buf_[0];
        d_buf_[0] = buf_now;
    }

    void pid::limitOut(double &val, const double limit) {
        if(limit < 0) return;
        else if(val > limit) val = limit;
        else if(val < -limit) val = -limit;
    }
}
