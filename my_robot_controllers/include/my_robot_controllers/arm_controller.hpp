#ifndef MY_ROBOT_CONTROLLERS__ARM_CONTROLLER_HPP
#define MY_ROBOT_CONTROLLERS__ARM_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using Float64MultiArray = std_msgs::msg::Float64MultiArray;

namespace arm_controller
{
  class ArmController : public controller_interface::ControllerInterface
  {
  public:
    ArmController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  protected:
    std::vector<std::string> joint_names_;
    std::string interface_name_;

    double coefficient_ = 1;

    std::vector<double> appCommand_;
    rclcpp::Subscription<Float64MultiArray>::SharedPtr pCommandSubscriber_;
  };
} // namespace arm_controller

#endif // MY_ROBOT_CONTROLLERS__ARM_CONTROLLER_HPP