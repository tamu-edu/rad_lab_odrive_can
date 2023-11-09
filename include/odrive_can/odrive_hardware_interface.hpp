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
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) override;
    hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> &, const std::vector<std::string> &) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  protected:
    // These hold the values that will actually be set/read by the controllers
    double cmd_pos_, cmd_vel_, cmd_effort_;
    double state_pos_, state_vel_, state_effort_, state_effort_target_;
    double state_bus_voltage_, state_bus_current_, state_fet_temp_, state_motor_temp_, state_disarm_reason_;
    double state_active_errors_, state_axis_state_, state_procedure_result_;
    double state_iq_setpoint_, state_iq_measured_;

    uint16_t node_id_;
    std::string can_interface_name_;
    SocketCanIntf can_intf_ = SocketCanIntf();
    EpollEventLoop event_loop_;
    std::thread event_loop_thread_;

    EpollEvent axis_state_event_;
    uint32_t axis_state_;
    std::mutex axis_state_mutex_;
    std::condition_variable fresh_heartbeat_;
    void set_axis_state();
    bool wait_for_axis_state_setting(uint32_t requested_state);

    void read_can_bus(const can_frame &frame);
    inline bool verify_length(const std::string &name, uint8_t expected, uint8_t length);

    std::mutex ctrl_stat_mutex_;
    odrive_can::msg::ControllerStatus ctrl_stat_ = odrive_can::msg::ControllerStatus();

    std::mutex odrv_stat_mutex_;
    odrive_can::msg::ODriveStatus odrv_stat_ = odrive_can::msg::ODriveStatus();

    enum ControlMode : std::uint8_t
    {
      UNDEFINED = 0,
      TORQUE = 1,
      VELOCITY = 2,
      POSITION = 3
    };

    ControlMode control_mode_ = ControlMode::UNDEFINED;

    // The ControlMode enum here matches the ODrive firmware control mode enum
    void write_control_mode(ControlMode mode);

    // The passed values must be in ODrive units (i.e. turns instead of radians)
    void write_command(ControlMode mode, const float &cmd_torque, const float &cmd_velocity, const float &cmd_position);
  };
} // namespace odrive_hardware_interface
