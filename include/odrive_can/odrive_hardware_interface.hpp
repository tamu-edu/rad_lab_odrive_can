#pragma once

#include "hardware_interface/actuator_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include "socket_can.hpp"

#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "odrive_can/msg/o_drive_status.hpp"

#include <algorithm>
#include <array>
#include <condition_variable>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <mutex>
#include <thread>

namespace odrive_hardware_interface
{
  class ODriveHardwareInterface : public hardware_interface::ActuatorInterface
  {
  public:
    // ODriveHardwareInterface();

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info);
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> &, const std::vector<std::string> &) override;
    hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> &, const std::vector<std::string> &) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  protected:
    // These hold the values that will actually be set/read by the controllers
    double axis_pos_cmd_, axis_vel_cmd_, axis_eff_cmd_;
    double axis_pos_state_, axis_vel_state_, axis_eff_state_;

    uint16_t node_id_;
    SocketCanIntf can_intf_ = SocketCanIntf();
    EpollEventLoop event_loop_;
    std::thread event_loop_thread_;

    void read_can_bus(const can_frame &frame);
    inline bool verify_length(const std::string &name, uint8_t expected, uint8_t length);

    std::mutex ctrl_stat_mutex_;
    odrive_can::msg::ControllerStatus ctrl_stat_ = odrive_can::msg::ControllerStatus();
    
    std::mutex odrv_stat_mutex_;
    odrive_can::msg::ODriveStatus odrv_stat_ = odrive_can::msg::ODriveStatus();

  };
} // namespace odrive_hardware_interface
