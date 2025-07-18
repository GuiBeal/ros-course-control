#ifndef MY_ROBOT_HARDWARE__ARM_HARDWARE_INTERFACE_HPP
#define MY_ROBOT_HARDWARE__ARM_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <my_robot_hardware/xl330_driver.hpp>

namespace arm_hardware
{
  class ArmHardwareInterface : public hardware_interface::SystemInterface
  {
  public:
    // Lifecycle Node methods
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

    // System Interface methods
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    std::shared_ptr<XL330Driver> pDriver_;
    int base_joint_motor_id_ = 1;
    int forearm_joint_motor_id_ = 2;
    std::string port_;
  };
} // namespace arm_hardware

#endif // MY_ROBOT_HARDWARE__ARM_HARDWARE_INTERFACE_HPP
