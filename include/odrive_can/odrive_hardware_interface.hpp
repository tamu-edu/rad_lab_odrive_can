#pragma once

#include "hardware_interface/actuator_interface.hpp"

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
    double fake_pos_cmd_, fake_pos_state_;
    double fake_vel_cmd_, fake_vel_state_;
    double fake_eff_cmd_, fake_eff_state_;
  };
} // namespace odrive_hardware_interface
