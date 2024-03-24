#ifndef HEXAPOD_KINEMATICS__KINEMATICS_HPP
#define HEXAPOD_KINEMATICS__KINEMATICS_HPP

#include "rclcpp/rclcpp.hpp"
#include "kdl/tree.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "std_msgs/msg/string.hpp"


namespace hexapod_kinematics
{
class Kinematics
{
public:
  Kinematics(const rclcpp::Node::SharedPtr& node);
  void registerFeet(
    const std::vector<std::string>& feet_links,
    const std::string& base_link="base_link");
  // TODO use more suitable type for goal_pos?
  int cartToJnt(
    const std::string& feet_link, const KDL::JntArray& q_init,
    const KDL::Frame& T_base_goal, KDL::JntArray& q_out);

private:
  void robot_description_callback(const std_msgs::msg::String& msg);
  void createSolvers();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  KDL::Tree tree_;
  std::vector<std::string> feet_links_;
  std::string base_link_;
  std::map<std::string, KDL::ChainIkSolverPos_LMA> solver_map_;
};
}   // namespace hexapod_kinematics

#endif  // HEXAPOD_KINEMATICS__KINEMATICS_HPP
