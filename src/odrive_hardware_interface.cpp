#include "byte_swap.hpp"
#include "odrive_can/odrive_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

using ODriveStatus = odrive_can::msg::ODriveStatus;
using ControllerStatus = odrive_can::msg::ControllerStatus;
using ControlMessage = odrive_can::msg::ControlMessage;

using std::placeholders::_1;

enum CmdId : uint32_t
{
  kHeartbeat = 0x001,            // ControllerStatus  - publisher
  kGetError = 0x003,             // SystemStatus      - publisher
  kSetAxisState = 0x007,         // SetAxisState      - service
  kGetEncoderEstimates = 0x009,  // ControllerStatus  - publisher
  kSetControllerMode = 0x00b,    // ControlMessage    - subscriber
  kSetInputPos,                  // ControlMessage    - subscriber
  kSetInputVel,                  // ControlMessage    - subscriber
  kSetInputTorque,               // ControlMessage    - subscriber
  kGetIq = 0x014,                // ControllerStatus  - publisher
  kGetTemp,                      // SystemStatus      - publisher
  kGetBusVoltageCurrent = 0x017, // SystemStatus      - publisher
  kGetTorques = 0x01c,           // ControllerStatus  - publisher
};

enum ControlMode : uint64_t
{
  kVoltageControl,
  kTorqueControl,
  kVelocityControl,
  kPositionControl,
};

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

    // TODO: set this proper
    node_id_ = 0;

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ODriveHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
  {
    // TODO: read can interface from params
    if (!can_intf_.init("can0", &event_loop_, std::bind(&ODriveHardwareInterface::read_can_bus, this, _1)))
    {
      RCLCPP_ERROR(rclcpp::get_logger("NODE_NAME_TODO"), "Failed to initialize socket can interface: can0");
      return CallbackReturn::ERROR;
    }

    event_loop_thread_ = std::thread([this]() { event_loop_.run_until_empty(); });

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ODriveHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
  {
    can_intf_.deinit();
    event_loop_thread_.join();
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // TODO: un-hardcode joint name
    state_interfaces.emplace_back(hardware_interface::StateInterface("joint1", "position", &axis_pos_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("joint1", "velocity", &axis_vel_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("joint1", "effort", &axis_eff_state_));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // TODO: un-hardcode joint name
    command_interfaces.emplace_back(hardware_interface::CommandInterface("joint1", "position", &axis_pos_cmd_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface("joint1", "velocity", &axis_vel_cmd_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface("joint1", "effort", &axis_eff_cmd_));

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
    std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
    axis_pos_state_ = ctrl_stat_.pos_estimate;
    axis_vel_state_ = ctrl_stat_.vel_estimate;
    axis_eff_state_ = ctrl_stat_.torque_estimate;

    return return_type::OK;
  }

  return_type ODriveHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    // Just set the state value to the command val for now
    // axis_pos_state_ = axis_pos_cmd_;
    // axis_vel_state_ = axis_vel_cmd_;
    // axis_eff_state_ = axis_eff_cmd_;

    return return_type::OK;
  }

  void ODriveHardwareInterface::read_can_bus(const can_frame &frame)
  {

    if (((frame.can_id >> 5) & 0x3F) != node_id_)
      return;

    switch (frame.can_id & 0x1F)
    {
    case CmdId::kHeartbeat:
    {
      if (!verify_length("kHeartbeat", 8, frame.len))
        break;
      std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
      ctrl_stat_.active_errors = read_le<uint32_t>(frame.data + 0);
      ctrl_stat_.axis_state = read_le<uint8_t>(frame.data + 4);
      ctrl_stat_.procedure_result = read_le<uint8_t>(frame.data + 5);
      ctrl_stat_.trajectory_done_flag = read_le<bool>(frame.data + 6);
      break;
    }
    case CmdId::kGetError:
    {
      if (!verify_length("kGetError", 8, frame.len))
        break;
      std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
      odrv_stat_.active_errors = read_le<uint32_t>(frame.data + 0);
      odrv_stat_.disarm_reason = read_le<uint32_t>(frame.data + 4);
      break;
    }
    case CmdId::kGetEncoderEstimates:
    {
      if (!verify_length("kGetEncoderEstimates", 8, frame.len))
        break;
      std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
      ctrl_stat_.pos_estimate = read_le<float>(frame.data + 0);
      ctrl_stat_.vel_estimate = read_le<float>(frame.data + 4);
      break;
    }
    case CmdId::kGetIq:
    {
      if (!verify_length("kGetIq", 8, frame.len))
        break;
      std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
      ctrl_stat_.iq_setpoint = read_le<float>(frame.data + 0);
      ctrl_stat_.iq_measured = read_le<float>(frame.data + 4);
      break;
    }
    case CmdId::kGetTemp:
    {
      if (!verify_length("kGetTemp", 8, frame.len))
        break;
      std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
      odrv_stat_.fet_temperature = read_le<float>(frame.data + 0);
      odrv_stat_.motor_temperature = read_le<float>(frame.data + 4);
      break;
    }
    case CmdId::kGetBusVoltageCurrent:
    {
      if (!verify_length("kGetBusVoltageCurrent", 8, frame.len))
        break;
      std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
      odrv_stat_.bus_voltage = read_le<float>(frame.data + 0);
      odrv_stat_.bus_current = read_le<float>(frame.data + 4);
      break;
    }
    case CmdId::kGetTorques:
    {
      if (!verify_length("kGetTorques", 8, frame.len))
        break;
      std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
      ctrl_stat_.torque_target = read_le<float>(frame.data + 0);
      ctrl_stat_.torque_estimate = read_le<float>(frame.data + 4);
      break;
    }
    default:
    {
      RCLCPP_WARN(rclcpp::get_logger("NODE_NAME_TODO"), "Received unused message: ID = 0x%x", (frame.can_id & 0x1F));
      break;
    }
    }
  }

  inline bool ODriveHardwareInterface::verify_length(const std::string &name, uint8_t expected, uint8_t length)
  {
    bool valid = expected == length;
    RCLCPP_DEBUG(rclcpp::get_logger("NODE_NAME_TODO"), "received %s", name.c_str());
    if (!valid)
      RCLCPP_WARN(rclcpp::get_logger("NODE_NAME_TODO"), "Incorrect %s frame length: %d != %d", name.c_str(), length, expected);
    return valid;
  }

} // namespace odrive_hardware_interface

PLUGINLIB_EXPORT_CLASS(odrive_hardware_interface::ODriveHardwareInterface, hardware_interface::ActuatorInterface)
