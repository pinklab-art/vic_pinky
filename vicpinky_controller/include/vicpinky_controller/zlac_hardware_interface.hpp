#ifndef ZLAC_HARDWARE_INTERFACE_HPP_
#define ZLAC_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zlac8015d.h"

namespace zlac_ros2_control
{
class ZlacHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ZlacHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::unique_ptr<ZLAC> zlac_driver_;

  // Store the command and state for the wheels
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  
  // Parameters from URDF
  std::string port_name_;
  int baud_rate_;
  int modbus_id_;
};
}  // namespace zlac_ros2_control

#endif  // ZLAC_HARDWARE_INTERFACE_HPP_