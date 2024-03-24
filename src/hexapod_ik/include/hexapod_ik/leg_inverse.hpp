#ifndef HEXAPOD_IK__LEG_INVERSE_HPP
#define HEXAPOD_IK__LEG_INVERSE_HPP

#include "rclcpp/rclcpp.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "std_msgs/msg/string.hpp"


namespace hexapod_ik
{
class LegInverse : public rclcpp::Node
{
public:
  LegInverse();

private:
  void robot_description_callback(const std_msgs::msg::String& msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::string urdf_;
  KDL::Tree tree_;
};
}   // namespace hexapod_ik

#endif  // HEXAPOD_IK__LEG_INVERSE_HPP
