#include "so101_hardware/so101_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace so101_hardware
{
hardware_interface::CallbackReturn So101Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  port_ = info_.hardware_parameters["port"];
  baudrate_ = std::stoi(info_.hardware_parameters["baudrate"]);
  timeout_ms_ = std::stoi(info_.hardware_parameters.at("timeout_ms"));

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // So101Hardware has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("So101Hardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("So101Hardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("So101Hardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("So101Hardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("So101Hardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  communication_initialized_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn So101Hardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize communication with the hardware
  RCLCPP_INFO(rclcpp::get_logger("So101Hardware"), "Configuring ...please wait...");

  if (!initCommunication())
  {
    RCLCPP_ERROR(rclcpp::get_logger("So101Hardware"), "Failed to initialize communication!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("So101Hardware"), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> So101Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> So101Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn So101Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("So101Hardware"), "Activating ...please wait...");

  // Set some default values when activating
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("So101Hardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn So101Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("So101Hardware"), "Deactivating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("So101Hardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type So101Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read sensor data from your hardware
  if (!readSensors(hw_positions_, hw_velocities_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("So101Hardware"), "Failed to read sensor data!");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type So101Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Send commands to your hardware
  if (!sendCommand(hw_commands_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("So101Hardware"), "Failed to send command!");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// TODO: Implement these methods based on your specific hardware communication
bool So101Hardware::initCommunication()
{
  // Initialize your communication interface (serial, CAN, Ethernet, etc.)
  // Example for serial communication:
  RCLCPP_INFO(rclcpp::get_logger("So101Hardware"), 
              "Initializing communication on port: %s with baudrate: %d", 
              port_.c_str(), baudrate_);
  
  // Add your hardware-specific initialization code here
  // Return true if successful, false otherwise
  
  communication_initialized_ = true;
  return true;
}

bool So101Hardware::sendCommand(const std::vector<double>& commands)
{
  if (!communication_initialized_)
  {
    return false;
  }

  // Send position commands to your hardware
  // This is where you implement the actual communication protocol
  
  // Example: Send each joint command
  for (size_t i = 0; i < commands.size(); ++i)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("So101Hardware"), 
                 "Sending command %f to joint %lu", commands[i], i);
    
    // Add your hardware-specific command sending code here
  }

  return true;
}

bool So101Hardware::readSensors(std::vector<double>& positions, std::vector<double>& velocities)
{
  if (!communication_initialized_)
  {
    return false;
  }

  // Read sensor data from your hardware
  // This is where you implement the actual sensor reading
  
  // Example: Read each joint position and velocity
  for (size_t i = 0; i < positions.size(); ++i)
  {
    // Add your hardware-specific sensor reading code here
    // For now, this is just a placeholder that maintains current values
    // positions[i] = read_position_from_hardware(i);
    // velocities[i] = read_velocity_from_hardware(i);
    
    RCLCPP_DEBUG(rclcpp::get_logger("So101Hardware"), 
                 "Read position %f and velocity %f from joint %lu", 
                 positions[i], velocities[i], i);
  }

  return true;
}

void So101Hardware::closeCommunication()
{
  if (communication_initialized_)
  {
    // Close your communication interface
    // Add your hardware-specific cleanup code here
    communication_initialized_ = false;
  }
}

}  // namespace so101_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(so101_hardware::So101Hardware, hardware_interface::SystemInterface)