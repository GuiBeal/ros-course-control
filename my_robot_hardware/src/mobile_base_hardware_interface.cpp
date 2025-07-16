#include <my_robot_hardware/mobile_base_hardware_interface.hpp>

namespace mobile_base_hardware
{
  hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    const auto ret = hardware_interface::SystemInterface::on_init(info);
    if (ret != hardware_interface::CallbackReturn::SUCCESS)
    {
      return ret;
    }

    left_motor_id_ = std::stoi(info_.hardware_parameters["left_motor_id"]);
    right_motor_id_ = std::stoi(info_.hardware_parameters["right_motor_id"]);
    port_ = info_.hardware_parameters["dynamixel_port"];

    pDriver_ = std::make_shared<XL330Driver>(port_);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure(const rclcpp_lifecycle::State &state)
  {
    if (pDriver_->init() != 0)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const auto &[name, description] : joint_command_interfaces_)
    {
      RCLCPP_DEBUG(get_logger(), "Command Interface name: %s", name.c_str());
    }
    for (const auto &[name, description] : joint_state_interfaces_)
    {
      RCLCPP_DEBUG(get_logger(), "State Interface name: %s", name.c_str());
    }

    return hardware_interface::SystemInterface::on_configure(state);
  }

  hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate(const rclcpp_lifecycle::State &state)
  {
    set_state("base_left_wheel_joint/velocity", 0.0);
    set_state("base_right_wheel_joint/velocity", 0.0);
    set_state("base_left_wheel_joint/position", 0.0);
    set_state("base_right_wheel_joint/position", 0.0);

    pDriver_->activateWithVelocityMode(left_motor_id_);
    pDriver_->activateWithVelocityMode(right_motor_id_);

    return hardware_interface::SystemInterface::on_activate(state);
  }

  hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    pDriver_->deactivate(left_motor_id_);
    pDriver_->deactivate(right_motor_id_);

    return hardware_interface::CallbackReturn::SUCCESS;

    return hardware_interface::SystemInterface::on_deactivate(state);
  }

  hardware_interface::return_type MobileBaseHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    double left_vel = pDriver_->getVelocityRadianPerSec(left_motor_id_);
    double right_vel = -pDriver_->getVelocityRadianPerSec(right_motor_id_); // swapped

    if (std::fabs(left_vel) < vel_tol_)
    {
      left_vel = 0;
    }
    if (std::fabs(right_vel) < vel_tol_)
    {
      right_vel = 0;
    }

    set_state("base_left_wheel_joint/velocity", left_vel);
    set_state("base_right_wheel_joint/velocity", right_vel);

    // const double left_pos = pDriver_->getPositionRadian(left_motor_id_);
    // const double right_pos = -pDriver_->getPositionRadian(right_motor_id_);
    // set_state("base_left_wheel_joint/position", left_pos);
    // set_state("base_left_wheel_joint/position", right_pos);

    set_state("base_left_wheel_joint/position", get_state("base_left_wheel_joint/position") + left_vel * period.seconds());
    set_state("base_right_wheel_joint/position", get_state("base_right_wheel_joint/position") + right_vel * period.seconds());

    RCLCPP_DEBUG(get_logger(), "READ - left_vel: %lf\tright_vel: %lf\tleft_pos: %lf\tright_pos: %lf",
                 left_vel, right_vel, get_state("base_left_wheel_joint/position"), get_state("base_right_wheel_joint/position"));

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MobileBaseHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    const auto left_vel = get_command("base_left_wheel_joint/velocity");
    const auto right_vel = -get_command("base_right_wheel_joint/velocity"); // swapped

    pDriver_->setTargetVelocityRadianPerSec(left_motor_id_, left_vel);
    pDriver_->setTargetVelocityRadianPerSec(right_motor_id_, right_vel);

    RCLCPP_DEBUG(get_logger(), "WRITE - left_vel: %lf\tright_vel: %lf", left_vel, right_vel);

    return hardware_interface::return_type::OK;
  }
} // namespace mobile_base_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface, hardware_interface::SystemInterface)
