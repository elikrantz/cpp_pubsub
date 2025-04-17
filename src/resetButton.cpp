// #include "cpp_pubsub/resetButton.hpp"
// #include <rviz_common/display_context.hpp>
// #include <rviz_common/viewport_mouse_event.hpp>
// #include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/string.hpp>
#include "cpp_pubsub/resetButton.hpp"
#include <std_msgs/msg/string.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>


namespace cpp_pubsub
{
  ResetButton::ResetButton()
  {
    shortcut_key_ = 'r';
  }

  void ResetButton::onInitialize()
  {
    auto raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    publisher_ = raw_node->create_publisher<std_msgs::msg::String>("/reset_button_topic", 10);
    RCLCPP_INFO(rclcpp::get_logger("ResetButton"), "Publisher initialized on /reset_button_topic");
  }

  int ResetButton::processMouseEvent(rviz_common::ViewportMouseEvent & event)
  {
    if (event.leftDown())
    {
      // std_msgs::msg::String msg;
      // msg.data = "Reset button clicked!";
      // publisher_->publish(msg);
      // RCLCPP_INFO(rclcpp::get_logger("ResetButton"), "Published: %s", msg.data.c_str());
      return Finished;
    }
    return Render;
  }

  // Override activate method to set tool active
  void ResetButton::activate()
  {
    tool_active_ = true;
    RCLCPP_INFO(rclcpp::get_logger("ResetButton"), "Activate");
    std_msgs::msg::String msg;
    msg.data = "Reset button clicked!";
    publisher_->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("ResetButton"), "Published: %s", msg.data.c_str());
  }

  // Override deactivate method to set tool inactive
  void ResetButton::deactivate()
  {
    tool_active_ = false;
  }
}

// #include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cpp_pubsub::ResetButton, rviz_common::Tool)
