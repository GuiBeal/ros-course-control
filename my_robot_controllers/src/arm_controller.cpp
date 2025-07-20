#include <my_robot_controllers/arm_controller.hpp>

namespace arm_controller
{
  ArmController::ArmController()
      : controller_interface::ControllerInterface()
  {
  }

  controller_interface::CallbackReturn ArmController::on_init()
  {
    joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
    interface_name_ = auto_declare<std::string>("interface_name", "position");
    coefficient_ = auto_declare<double>("coefficient", 0.8);

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ArmController::on_configure(const rclcpp_lifecycle::State &state)
  {
    auto cbCommand = [this](const Float64MultiArray::SharedPtr msg) -> void
    {
      if (msg->data.size() == joint_names_.size())
      {
        appCommand_ = msg->data;
      }
      else
      {
        RCLCPP_WARN(get_node()->get_logger(), "Expected command with size %lu, received command with size %lu",
                    joint_names_.size(), msg->data.size());
      }
    };

    pCommandSubscriber_ = get_node()->create_subscription<Float64MultiArray>("/joints_command", 10, cbCommand);

    return controller_interface::ControllerInterface::on_configure(state);
  }

  controller_interface::InterfaceConfiguration ArmController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(joint_names_.size());
    for (const std::string& joint_name : joint_names_)
    {
      config.names.push_back(joint_name + "/" + interface_name_);
    }
    return config;
  }

  controller_interface::InterfaceConfiguration ArmController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(joint_names_.size());
    for (const std::string& joint_name : joint_names_)
    {
      config.names.push_back(joint_name + "/" + interface_name_);
    }
    return config;
  }

  controller_interface::CallbackReturn ArmController::on_activate(const rclcpp_lifecycle::State &state)
  {
    appCommand_.clear();
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      appCommand_.push_back(state_interfaces_[i].get_optional().value());
    }

    return controller_interface::ControllerInterface::on_activate(state);
  }

  controller_interface::return_type ArmController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      const double state = state_interfaces_[i].get_optional().value();
      const double &command = appCommand_[i];

      const double updatedCommand = coefficient_ * command + (1 - coefficient_) * state;

      std::ignore = command_interfaces_[i].set_value(updatedCommand);
    }

    return controller_interface::return_type::OK;
  }
} // namespace arm_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arm_controller::ArmController, controller_interface::ControllerInterface);