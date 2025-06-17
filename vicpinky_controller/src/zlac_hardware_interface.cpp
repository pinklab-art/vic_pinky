#include "zlac_ros2_control/zlac_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <cmath>
#include <vector>
#include <string>
#include <limits>

// 상수 정의
const double RPM2RAD = M_PI / 30.0;

namespace zlac_ros2_control
{

hardware_interface::CallbackReturn ZlacHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // URDF에서 파라미터 읽기
  port_name_ = info_.hardware_parameters["port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  modbus_id_ = std::stoi(info_.hardware_parameters["modbus_id"]);

  RCLCPP_INFO(rclcpp::get_logger("ZlacHardwareInterface"), "Port: %s, Baud: %d, ID: %d", port_name_.c_str(), baud_rate_, modbus_id_);

  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);

  zlac_driver_ = std::make_unique<ZLAC>();
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ZlacHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "position", &hw_positions_[i]));
    
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "velocity", &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ZlacHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "velocity", &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn ZlacHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ZlacHardwareInterface"), "Activating ...please wait...");

  try {
    zlac_driver_->begin(port_name_, baud_rate_, (uint8_t)modbus_id_);
    zlac_driver_->set_vel_mode();
    if (zlac_driver_->enable() != 0) {
        RCLCPP_FATAL(rclcpp::get_logger("ZlacHardwareInterface"), "Failed to enable motors.");
        return hardware_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("ZlacHardwareInterface"), "Failed to open serial port %s. Error: %s", port_name_.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("ZlacHardwareInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZlacHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ZlacHardwareInterface"), "Deactivating ...please wait...");
  // 정지 명령을 보낸 후 비활성화
  zlac_driver_->set_double_rpm(0, 0);
  zlac_driver_->disable();
  RCLCPP_INFO(rclcpp::get_logger("ZlacHardwareInterface"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ZlacHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  MOT_DATA motor_feedback = zlac_driver_->get_rpm();

  // Joint 0: Left Wheel, Joint 1: Right Wheel
  double vel_l_rads = motor_feedback.rpm_L * RPM2RAD; 
  hw_velocities_[0] = vel_l_rads;
  hw_positions_[0] += vel_l_rads * period.seconds();

  double vel_r_rads = motor_feedback.rpm_R * RPM2RAD; 
  hw_velocities_[1] = vel_r_rads;
  hw_positions_[1] += vel_r_rads * period.seconds();
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ZlacHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double cmd_l = std::isnan(hw_commands_[0]) ? 0.0 : hw_commands_[0];
  double cmd_r = std::isnan(hw_commands_[1]) ? 0.0 : hw_commands_[1];

  double rpm_L = cmd_l / RPM2RAD;
  double rpm_R = cmd_r / RPM2RAD;
  
  zlac_driver_->set_double_rpm((int16_t)rpm_L, (int16_t)rpm_R);

  return hardware_interface::return_type::OK;
}

}  // namespace zlac_ros2_control

PLUGINLIB_EXPORT_CLASS(
  zlac_ros2_control::ZlacHardwareInterface,
  hardware_interface::SystemInterface)
