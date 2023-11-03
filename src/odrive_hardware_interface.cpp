#include "odrive_can/odrive_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace odrive_hardware_interface
{
  CallbackReturn ODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
  {
    // Call parent's init
    auto init_result = hardware_interface::ActuatorInterface::on_init(hardware_info);
    if (init_result != CallbackReturn::SUCCESS)
    {
      return init_result;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ODriveHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ODriveHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
  {
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface("joint1", "position", &fake_pos_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("joint1", "velocity", &fake_vel_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("joint1", "effort", &fake_eff_state_));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface("joint1", "position", &fake_pos_cmd_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface("joint1", "velocity", &fake_vel_cmd_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface("joint1", "effort", &fake_eff_cmd_));

    return command_interfaces;
  }

  return_type ODriveHardwareInterface::prepare_command_mode_switch(const std::vector<std::string> &, const std::vector<std::string> &)
  {
    return return_type::OK;
  }

  return_type ODriveHardwareInterface::perform_command_mode_switch(const std::vector<std::string> &, const std::vector<std::string> &)
  {
    return return_type::OK;
  }

  CallbackReturn ODriveHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ODriveHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ODriveHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &previous_state)
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ODriveHardwareInterface::on_error(const rclcpp_lifecycle::State &previous_state)
  {
    return CallbackReturn::SUCCESS;
  }

  return_type ODriveHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    return return_type::OK;
  }

  return_type ODriveHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    // Just set the state value to the command val for now
    fake_pos_state_ = fake_pos_cmd_;
    fake_vel_state_ = fake_vel_cmd_;
    fake_eff_state_ = fake_eff_cmd_;
    
    return return_type::OK;
  }

} // namespace odrive_hardware_interface

PLUGINLIB_EXPORT_CLASS(odrive_hardware_interface::ODriveHardwareInterface, hardware_interface::ActuatorInterface)
