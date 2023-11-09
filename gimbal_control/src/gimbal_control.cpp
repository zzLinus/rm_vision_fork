//
// Created by rhine on 7/16/23.
//

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gimbal_control.hpp"
#include "pid.hpp"
#include "rm_msgs/msg/debug_msg.hpp"
#include "rm_msgs/msg/detector_msg.hpp"
#include "rm_msgs/msg/serial_send.hpp"

using namespace std::chrono_literals;

namespace gimbal {
GimbalControl::GimbalControl(const rclcpp::NodeOptions &options)
    : Node("gimbal_control", options) {
  serial_pub_ =
      this->create_publisher<rm_msgs::msg::SerialSend>("/serial/send", 10);

  last_msg_time = std::chrono::system_clock::now() - 300ms;

  pwm_count[0] = base_count_x;
  pwm_count[1] = base_count_y;

  speed = 1.0;

  point_pid_config_x_ = std::make_unique<pid_config>(getPidParams("x"));
  point_pid_config_y_ = std::make_unique<pid_config>(getPidParams("y"));

  point_x_pid_ = std::make_unique<pid>(*point_pid_config_x_);
  point_y_pid_ = std::make_unique<pid>(*point_pid_config_y_);

  debug_pub_ =
      this->create_publisher<rm_msgs::msg::DebugMsg>("/gimbal/debug", 10);

  light_point_sub_ = this->create_subscription<rm_msgs::msg::DetectorMsg>(
      "/detector/light_point", 10,
      std::bind(&GimbalControl::midPointCallback, this, std::placeholders::_1));

  gimbal_timer_ = this->create_wall_timer(
      10ms, std::bind(&GimbalControl::updateGimbal, this));
}

pid_config GimbalControl::getPidParams(const std::string &pid_name) {

  double kp, ki, kd, i_max, out_max;
  PID_MODE mode;

  try {
    kp = declare_parameter<double>("pid_kp_" + pid_name, 0.0);
  } catch (rclcpp::ParameterTypeException &ex) {
    RCLCPP_ERROR(get_logger(), "The pid_kp provided was invalid");
    throw ex;
  }

  try {
    ki = declare_parameter<double>("pid_ki_" + pid_name, 0.0);
  } catch (rclcpp::ParameterTypeException &ex) {
    RCLCPP_ERROR(get_logger(), "The pid_ki provided was invalid");
    throw ex;
  }

  try {
    kd = declare_parameter<double>("pid_kd_" + pid_name, 0.0);
  } catch (rclcpp::ParameterTypeException &ex) {
    RCLCPP_ERROR(get_logger(), "The pid_kd provided was invalid");
    throw ex;
  }

  try {
    i_max = declare_parameter<double>("pid_i_max_" + pid_name, -1.0);
  } catch (rclcpp::ParameterTypeException &ex) {
    RCLCPP_INFO(get_logger(), "PID integral value unlimited");
    throw ex;
  }

  try {
    out_max = declare_parameter<double>("pid_out_max_" + pid_name, -1.0);
  } catch (rclcpp::ParameterTypeException &ex) {
    RCLCPP_ERROR(get_logger(), "PID output value unlimited");
    throw ex;
  }

  try {
    const auto mode_string =
        declare_parameter<std::string>("pid_mode_" + pid_name, "");

    RCLCPP_INFO(get_logger(), "pid_mode: %s\n", mode_string.c_str());

    if (mode_string == "position") {
      mode = PID_MODE::PID_POSITION;
    } else if (mode_string == "delta") {
      mode = PID_MODE::PID_DELTA;
    } else {
      throw std::invalid_argument{
          "The pid_mode parameter must be one of: position, or delta."};
    }
  } catch (rclcpp::ParameterTypeException &ex) {
    RCLCPP_ERROR(get_logger(), "The pid_mode provided was invalid");
    throw ex;
  }

  RCLCPP_INFO(get_logger(),
              "pid of %s: %lf, %lf, %lf\ni_max: %lf\nout_max: %lf\nmode: %s",
              pid_name.c_str(), kp, ki, kd, i_max, out_max,
              mode == PID_MODE::PID_DELTA ? "delta" : "position");

  return pid_config(kp, ki, kd, i_max, out_max, mode);
}

void GimbalControl::midPointCallback(const rm_msgs::msg::DetectorMsg &msg) {
  if (msg.light_point_x >= 0 && msg.target_point_x >= 0) {
    light_point_ = {msg.light_point_x, msg.light_point_y};
    target_point_ = {msg.target_point_x, msg.target_point_y};
    last_msg_time = std::chrono::system_clock::now();
  }
}

void GimbalControl::updateGimbal() {
  if ((std::chrono::system_clock::now() - last_msg_time) < 50ms) {
    if (now_target_point_ == std::pair<double, double>{0.0, 0.0}) {
      now_target_point_ = target_point_;
    } else {
      auto dx = std::abs(target_point_.first - now_target_point_.first);
      auto dy = std::abs(target_point_.second - now_target_point_.second);

      auto vx = speed * cos(atan2(dy, dx));
      auto vy = speed * sin(atan2(dy, dx));

      if (std::abs(now_target_point_.first - target_point_.first) < vx) {
        now_target_point_.first = target_point_.first;
      } else {
        now_target_point_.first +=
            (target_point_.first > now_target_point_.first ? vx : -vx);
      }

      if (std::abs(now_target_point_.second - target_point_.second) < vy) {
        now_target_point_.second = target_point_.second;
      } else {
        now_target_point_.second +=
            (target_point_.second > now_target_point_.second ? vy : -vy);
      }
    }
    if (std::abs(light_point_.first - now_target_point_.first) < 4) {
      point_x_pid_->clear();
    } else {
      auto pid_out_x = point_x_pid_->calc(-light_point_.first / 100.0,
                                          -now_target_point_.first / 100.0);
      pwm_count[0] = base_count_x + pid_out_x * regulate_count_x;
    }
    if (std::abs(light_point_.second - now_target_point_.second) < 4) {
      point_y_pid_->clear();
    } else {
      auto pid_out_y = point_y_pid_->calc(-light_point_.second / 100.0,
                                          -now_target_point_.second / 100.0);
      pwm_count[1] = base_count_y + pid_out_y * regulate_count_y;
    }

    rm_msgs::msg::DebugMsg debug_msg;
    debug_msg.now_target_point_x = (int)now_target_point_.first;
    debug_msg.now_target_point_y = (int)now_target_point_.second;
    debug_pub_->publish(debug_msg);

    std::cout << "point_x: " << light_point_.first
              << ", point_y: " << light_point_.second << std::endl;
    std::cout << "target_x: " << target_point_.first
              << ", target_y: " << target_point_.second << std::endl;
  } else {
    point_x_pid_->clear();
    point_y_pid_->clear();
  }
  rm_msgs::msg::SerialSend msg;
  msg.header = 0x5A;
  msg.crc_need = false;
  msg.data.resize(sizeof(pwm_count));
  std::copy(reinterpret_cast<const uint8_t *>(pwm_count),
            reinterpret_cast<const uint8_t *>(pwm_count + 2), msg.data.begin());
  //        RCLCPP_INFO(get_logger(), "pwm_count: %d %d", pwm_count[0],
  //        pwm_count[1]);
  serial_pub_->publish(msg);
}
} // namespace gimbal

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gimbal::GimbalControl)
