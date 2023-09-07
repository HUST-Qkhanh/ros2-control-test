// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AMR_ROBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_
#define AMR_ROBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
// #include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "amr_robot_hardware/visibility_control.h"
#include "protocol.h"
#include "pid.h"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include "config.h"

constexpr double WHEEL_RADIUS = 0.08255;
constexpr double MAX_VELOCITY = 1;
constexpr int DIRECTION_CORRECTION = -1;
constexpr char PORT[] = "/dev/hoverboard";

namespace amr_robot_hardware
{
  class DiffBotSystemHardware
      : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

    DiffBotSystemHardware();
    ~DiffBotSystemHardware()
    {
      if (port_fd != -1)
        close(port_fd);
    }

    AMR_ROBOT_HARDWARE_PUBLIC
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    AMR_ROBOT_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    AMR_ROBOT_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    AMR_ROBOT_HARDWARE_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    AMR_ROBOT_HARDWARE_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    AMR_ROBOT_HARDWARE_PUBLIC
    hardware_interface::return_type read() override;

    AMR_ROBOT_HARDWARE_PUBLIC
    hardware_interface::return_type write() override;

    void protocol_recv(char byte);
    void on_encoder_update(int16_t right, int16_t left);

  private:
    // Parameters for the DiffBot simulation
    double hw_start_sec_;
    double hw_stop_sec_;

    // Store the wheeled robot position
    double base_x_, base_y_, base_theta_;
    // The units for wheels are radians (pos), radians per second (vel,cmd), and Netwton metres (eff)
    struct Joint
    {
      std_msgs::msg::Float64 pos;
      std_msgs::msg::Float64 vel;
      std_msgs::msg::Float64 eff;
      std_msgs::msg::Float64 cmd;
    } joints[2];

    // Publishers
    rclcpp::Node node;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> vel_pub[2];
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> pos_pub[2];
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> cmd_pub[2];
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> voltage_pub;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> temp_pub;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> connected_pub;

    double wheel_radius = WHEEL_RADIUS;
    double max_velocity = MAX_VELOCITY;
    int direction_correction = DIRECTION_CORRECTION;
    std::string port = PORT;

    rclcpp::Time last_read;
    // Last known encoder values
    int16_t last_wheelcountR;
    int16_t last_wheelcountL;
    // Count of full encoder wraps
    int multR;
    int multL;
    // Thresholds for calculating the wrap
    int low_wrap;
    int high_wrap;

    // Hoverboard protocol
    int port_fd;
    long unsigned int msg_len = 0;
    char prev_byte = 0;
    uint16_t start_frame = 0;
    char *p;
    SerialFeedback msg, prev_msg;

    PID pids[2];
  };

} // namespace amr_robot_hardware

#endif // AMR_ROBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_
