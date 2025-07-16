#ifndef MY_ROBOT_HARDWARE__MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MY_ROBOT_HARDWARE__MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <my_robot_hardware/xl330_driver.hpp>

namespace mobile_base_hardware
{
  class MobileBaseHardwareInterface : public hardware_interface::SystemInterface
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
    int left_motor_id_ = 1;
    int right_motor_id_ = 2;
    std::string port_;

    const double vel_tol_ = 0.03;
  };
} // namespace mobile_base_hardware

#endif // MY_ROBOT_HARDWARE__MOBILE_BASE_HARDWARE_INTERFACE_HPP
