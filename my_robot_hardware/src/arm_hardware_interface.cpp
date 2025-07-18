#include <my_robot_hardware/arm_hardware_interface.hpp>

namespace arm_hardware
{
  hardware_interface::CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    const auto ret = hardware_interface::SystemInterface::on_init(info);
    if (ret != hardware_interface::CallbackReturn::SUCCESS)
    {
      return ret;
    }

    base_joint_motor_id_ = std::stoi(info_.hardware_parameters["base_joint_motor_id"]);
    forearm_joint_motor_id_ = std::stoi(info_.hardware_parameters["forearm_joint_motor_id"]);
    port_ = info_.hardware_parameters["dynamixel_port"];

    pDriver_ = std::make_shared<XL330Driver>(port_);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ArmHardwareInterface::on_configure(const rclcpp_lifecycle::State &state)
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

  hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State &state)
  {
    // set_state("arm_base_forearm_joint/position", 0.0);
    // set_state("forearm_hand_joint/position", 0.0);

    pDriver_->activateWithPositionMode(base_joint_motor_id_);
    pDriver_->activateWithPositionMode(forearm_joint_motor_id_);

    return hardware_interface::SystemInterface::on_activate(state);
  }

  hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    pDriver_->deactivate(base_joint_motor_id_);
    pDriver_->deactivate(forearm_joint_motor_id_);

    return hardware_interface::CallbackReturn::SUCCESS;

    return hardware_interface::SystemInterface::on_deactivate(state);
  }

  hardware_interface::return_type ArmHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    const double base_pos = pDriver_->getPositionRadian(base_joint_motor_id_);
    const double forearm_pos = pDriver_->getPositionRadian(forearm_joint_motor_id_);

    set_state("arm_base_forearm_joint/position", base_pos);
    set_state("forearm_hand_joint/position", forearm_pos);

    RCLCPP_DEBUG(get_logger(), "READ - base pos: %lf\tforearm pos: %lf", base_pos, forearm_pos);

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type ArmHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    const auto base_pos = get_command("arm_base_forearm_joint/position");
    const auto forearm_pos = get_command("forearm_hand_joint/position");

    pDriver_->setTargetPositionRadian(base_joint_motor_id_, base_pos);
    pDriver_->setTargetPositionRadian(forearm_joint_motor_id_, forearm_pos);

    RCLCPP_DEBUG(get_logger(), "WRITE - base pos: %lf\tforearm pos: %lf", base_pos, forearm_pos);

    return hardware_interface::return_type::OK;
  }
} // namespace arm_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmHardwareInterface, hardware_interface::SystemInterface)
