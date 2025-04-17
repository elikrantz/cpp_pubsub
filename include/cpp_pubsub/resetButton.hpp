#ifndef CPP_PUBSUB__RESET_BUTTON_HPP_
#define CPP_PUBSUB__RESET_BUTTON_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

namespace cpp_pubsub
{

class ResetButton : public rviz_common::Tool
{
public:
  ResetButton();
  ~ResetButton() override = default;

  void onInitialize() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
  void activate() override;
  void deactivate() override;

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  bool tool_active_;
};

}  // namespace cpp_pubsub

#endif  // CPP_PUBSUB__RESET_BUTTON_HPP_
