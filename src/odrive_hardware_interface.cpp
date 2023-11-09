#include "byte_swap.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_can/odrive_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <chrono>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

using ODriveStatus = odrive_can::msg::ODriveStatus;
using ControllerStatus = odrive_can::msg::ControllerStatus;
using ControlMessage = odrive_can::msg::ControlMessage;

using std::placeholders::_1;
using namespace std::chrono_literals;

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
  ODriveHardwareInterface::~ODriveHardwareInterface()
  {
    on_deactivate(rclcpp_lifecycle::State());
    on_cleanup(rclcpp_lifecycle::State());
  }

  CallbackReturn ODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
  {
    // Call parent's init
    auto init_result = hardware_interface::ActuatorInterface::on_init(hardware_info);
    if (init_result != CallbackReturn::SUCCESS)
    {
      return init_result;
    }

    // Get parameters from URDF ros2_control macro
    node_id_ = std::stoi(info_.hardware_parameters["node_id"]);
    can_interface_name_ = info_.hardware_parameters["can_interface"];

    // Make sure we only have one joint
    if (info_.joints.size() != 1)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "Expected 1 joint, got: %zu", info_.joints.size());
      return CallbackReturn::ERROR;
    }

    // Make sure we have position, velocity, and torque command and state interface
    auto &joint_cmd_IFs = info_.joints.front().command_interfaces;
    auto &joint_state_IFs = info_.joints.front().state_interfaces;
    auto interface_to_find = hardware_interface::HW_IF_POSITION;
    auto find_interface = [&interface_to_find](hardware_interface::InterfaceInfo info)
    {
      return info.name == interface_to_find;
    };

    bool missing_interface = false;
    std::string warning_msg = "Missing interfaces. Please add these to the ros2_control tag of your URDF:\n";

    // Search for position interfaces
    auto it = std::find_if(joint_cmd_IFs.begin(), joint_cmd_IFs.end(), find_interface);
    if (it == joint_cmd_IFs.end())
    {
      missing_interface = true;
      warning_msg += "Command Interface: position\n";
    }
    it = std::find_if(joint_state_IFs.begin(), joint_state_IFs.end(), find_interface);
    if (it == joint_state_IFs.end())
    {
      missing_interface = true;
      warning_msg += "State Interface: position\n";
    }

    // Search for velocity interfaces
    interface_to_find = hardware_interface::HW_IF_VELOCITY;
    it = std::find_if(joint_cmd_IFs.begin(), joint_cmd_IFs.end(), find_interface);
    if (it == joint_cmd_IFs.end())
    {
      missing_interface = true;
      warning_msg += "Command Interface: velocity\n";
    }
    it = std::find_if(joint_state_IFs.begin(), joint_state_IFs.end(), find_interface);
    if (it == joint_state_IFs.end())
    {
      missing_interface = true;
      warning_msg += "State Interface: velocity\n";
    }

    // Search for effort interfaces
    interface_to_find = hardware_interface::HW_IF_EFFORT;
    it = std::find_if(joint_cmd_IFs.begin(), joint_cmd_IFs.end(), find_interface);
    if (it == joint_cmd_IFs.end())
    {
      missing_interface = true;
      warning_msg += "Command Interface: effort\n";
    }
    it = std::find_if(joint_state_IFs.begin(), joint_state_IFs.end(), find_interface);
    if (it == joint_state_IFs.end())
    {
      missing_interface = true;
      warning_msg += "State Interface: effort\n";
    }

    if (missing_interface)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("ODriveHardwareInterface"), warning_msg);
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ODriveHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
  {
    if (!can_intf_.init(can_interface_name_, &event_loop_, std::bind(&ODriveHardwareInterface::read_can_bus, this, _1)))
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("ODriveHardwareInterface"), "Failed to initialize socket can interface: " << can_interface_name_);
      return CallbackReturn::ERROR;
    }

    if (!axis_state_event_.init(&event_loop_, std::bind(&ODriveHardwareInterface::set_axis_state, this)))
    {
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "Failed to initialize set axis state event");
      return CallbackReturn::ERROR;
    }

    event_loop_thread_ = std::thread([this]()
                                     { event_loop_.run_until_empty(); });

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ODriveHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
  {
    axis_state_event_.deinit();
    can_intf_.deinit();
    if (event_loop_thread_.joinable())
    {
      event_loop_thread_.join();
    }
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Normal position, velocity, and torque states
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.front().name, hardware_interface::HW_IF_POSITION, &state_pos_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.front().name, hardware_interface::HW_IF_VELOCITY, &state_vel_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.front().name, hardware_interface::HW_IF_EFFORT, &state_effort_));

    // Provide all other information we get from ODrive (voltage, errors, etc)
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.front().name, "effort_target", &state_effort_target_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.front().name, "bus_voltage", &state_bus_voltage_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.front().name, "bus_current", &state_bus_current_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.front().name, "fet_temp", &state_fet_temp_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.front().name, "motor_temp", &state_motor_temp_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.front().name, "disarm_reason", &state_disarm_reason_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.front().name, "active_errors", &state_active_errors_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.front().name, "axis_state", &state_axis_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.front().name, "procedure_result", &state_procedure_result_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.front().name, "iq_setpoint", &state_iq_setpoint_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.front().name, "iq_measured", &state_iq_measured_));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints.front().name, hardware_interface::HW_IF_POSITION, &cmd_pos_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints.front().name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints.front().name, hardware_interface::HW_IF_EFFORT, &cmd_effort_));

    return command_interfaces;
  }

  return_type ODriveHardwareInterface::prepare_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
  {
    // Set any stopping interface to UNDEFINED control mode
    for (auto key : stop_interfaces)
    {
      if (key.find(info_.joints.front().name) != std::string::npos)
      {
        control_mode_ = ControlMode::UNDEFINED;
      }
    }

    // Set starting interface accordingly
    ControlMode new_control_mode = ControlMode::UNDEFINED;
    for (auto key : start_interfaces)
    {
      if (key == info_.joints.front().name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        new_control_mode = ControlMode::TORQUE;
      }
      else if (key == info_.joints.front().name + "/" + hardware_interface::HW_IF_POSITION)
      {
        new_control_mode = ControlMode::POSITION;
      }
      else if (key == info_.joints.front().name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        new_control_mode = ControlMode::VELOCITY;
      }
    }

    // Make sure we aren't setting a new control mode if another controller has claimed this interface already
    if (new_control_mode != ControlMode::UNDEFINED && control_mode_ != ControlMode::UNDEFINED)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("ODriveHardwareInterface"), "Tried to claim already claimed hardware interface");
      return return_type::ERROR;
    }

    // Set new control mode
    control_mode_ = new_control_mode;

    return return_type::OK;
  }

  return_type ODriveHardwareInterface::perform_command_mode_switch(const std::vector<std::string> &, const std::vector<std::string> &)
  {
    if (control_mode_ == ControlMode::UNDEFINED)
    {
      // Set axis to idle? This will essentially turn off the motor
      // TODO think about if we actually want this
      return (on_deactivate(rclcpp_lifecycle::State()) == CallbackReturn::SUCCESS) ? return_type::OK : return_type::ERROR;
    }

    // Set axis to closed loop control mode
    if (on_activate(rclcpp_lifecycle::State()) != CallbackReturn::SUCCESS)
    {
      return return_type::ERROR;
    }

    // Set axis control mode
    write_control_mode(control_mode_);

    // Pull the current values
    float current_torque, current_vel, current_pos;
    {
      std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
      current_torque = ctrl_stat_.torque_estimate;
      current_vel = ctrl_stat_.vel_estimate;
      current_pos = ctrl_stat_.pos_estimate;
    }

    // Write current values to motor commands (i.e. hold current position, velocity, torque)
    // But, set all feedforward terms to 0
    switch (control_mode_)
    {
    case ControlMode::TORQUE:
      cmd_effort_ = current_torque;
      write_command(control_mode_, current_torque, 0, 0);
      break;
    case ControlMode::VELOCITY:
      cmd_vel_ = current_vel;
      cmd_effort_ = 0;
      write_command(control_mode_, 0, current_vel, 0);
      break;
    case ControlMode::POSITION:
      cmd_pos_ = current_pos;
      cmd_effort_ = 0;
      cmd_vel_ = 0;
      write_command(control_mode_, 0, 0, current_pos);
      break;
    default:
      break;
    }
    return return_type::OK;
  }

  CallbackReturn ODriveHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
  {
    constexpr uint32_t CLOSED_LOOP_CONTROL_STATE = 8;
    {
      std::unique_lock<std::mutex> guard(axis_state_mutex_);
      axis_state_ = CLOSED_LOOP_CONTROL_STATE;
    }
    axis_state_event_.set();

    if (!wait_for_axis_state_setting(CLOSED_LOOP_CONTROL_STATE))
    {
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "Failed to set axis to CLOSED_LOOP_CONTROL");
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ODriveHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
  {
    constexpr uint32_t IDLE_STATE = 1;
    {
      std::unique_lock<std::mutex> guard(axis_state_mutex_);
      axis_state_ = IDLE_STATE;
    }
    axis_state_event_.set();

    if (!wait_for_axis_state_setting(IDLE_STATE))
    {
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "Failed to set axis to IDLE");
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ODriveHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ODriveHardwareInterface::on_error(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  return_type ODriveHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    {
      std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
      // Position/velocity come from ODrive in turns, convert to radians
      state_pos_ = ctrl_stat_.pos_estimate * 2 * M_PI;
      state_vel_ = ctrl_stat_.vel_estimate * 2 * M_PI;
      state_effort_ = ctrl_stat_.torque_estimate;
      state_effort_target_ = ctrl_stat_.torque_target;
      state_axis_state_ = ctrl_stat_.axis_state;
      state_procedure_result_ = ctrl_stat_.procedure_result;
      state_iq_setpoint_ = ctrl_stat_.iq_setpoint;
      state_iq_measured_ = ctrl_stat_.iq_measured;
    }

    {
      std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
      state_bus_voltage_ = odrv_stat_.bus_voltage;
      state_bus_current_ = odrv_stat_.bus_current;
      state_fet_temp_ = odrv_stat_.fet_temperature;
      state_motor_temp_ = odrv_stat_.motor_temperature;
      state_disarm_reason_ = odrv_stat_.disarm_reason;
      state_active_errors_ = odrv_stat_.active_errors;
    }

    return return_type::OK;
  }

  return_type ODriveHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    // All position/velocity commands go to the ODrive in turns instead of radians
    write_command(control_mode_, cmd_effort_, cmd_vel_ / 2 / M_PI, cmd_pos_ / 2 / M_PI);

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
      fresh_heartbeat_.notify_one();
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
      RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "Received unused message: ID = 0x%x", (frame.can_id & 0x1F));
      break;
    }
    }
  }

  inline bool ODriveHardwareInterface::verify_length(const std::string &name, uint8_t expected, uint8_t length)
  {
    bool valid = expected == length;
    RCLCPP_DEBUG(rclcpp::get_logger("ODriveHardwareInterface"), "received %s", name.c_str());
    if (!valid)
      RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "Incorrect %s frame length: %d != %d", name.c_str(), length, expected);
    return valid;
  }

  void ODriveHardwareInterface::set_axis_state()
  {
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
    {
      std::unique_lock<std::mutex> guard(axis_state_mutex_);
      write_le<uint32_t>(axis_state_, frame.data);
    }
    frame.len = 4;
    can_intf_.send_can_frame(frame);
  }

  bool ODriveHardwareInterface::wait_for_axis_state_setting(uint32_t requested_state)
  {
    std::unique_lock<std::mutex> guard(ctrl_stat_mutex_); // define lock for controller status
    return fresh_heartbeat_.wait_for(guard, 100ms, [this, &requested_state](){return ctrl_stat_.axis_state == requested_state;});
  }

  void ODriveHardwareInterface::write_control_mode(ControlMode mode)
  {
    if (mode == ControlMode::UNDEFINED)
    {
      return;
    }

    struct can_frame frame = can_frame{};
    frame.can_id = node_id_ << 5 | kSetControllerMode;
    write_le<uint32_t>(mode, frame.data);
    write_le<uint32_t>(1, frame.data + 4); // Passthrough input
    frame.len = 8;
    can_intf_.send_can_frame(frame);
  }

  void ODriveHardwareInterface::write_command(ControlMode mode, const float &cmd_torque, const float &cmd_velocity, const float &cmd_position)
  {
    if (mode == ControlMode::UNDEFINED)
    {
      return;
    }

    struct can_frame frame = can_frame{};
    switch (mode)
    {
    case ControlMode::TORQUE:
      frame.can_id = node_id_ << 5 | kSetInputTorque;
      write_le<float>(cmd_torque, frame.data);
      frame.len = 4;
      break;
    case ControlMode::VELOCITY:
      frame.can_id = node_id_ << 5 | kSetInputVel;
      write_le<float>(cmd_velocity, frame.data);
      write_le<float>(cmd_torque, frame.data + 4);
      frame.len = 8;
      break;
    case ControlMode::POSITION:
      frame.can_id = node_id_ << 5 | kSetInputPos;
      write_le<float>(cmd_position, frame.data);
      write_le<int8_t>(((int8_t)((cmd_velocity) * 1000)), frame.data + 4);
      write_le<int8_t>(((int8_t)((cmd_torque) * 1000)), frame.data + 6);
      frame.len = 8;
      break;
    default:
      return;
    }

    can_intf_.send_can_frame(frame);
  }

} // namespace odrive_hardware_interface

PLUGINLIB_EXPORT_CLASS(odrive_hardware_interface::ODriveHardwareInterface, hardware_interface::ActuatorInterface)
