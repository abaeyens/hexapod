#ifndef HEXAPOD_KINEMATICS__KINEMATICS_HPP_
#define HEXAPOD_KINEMATICS__KINEMATICS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "kdl/tree.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "std_msgs/msg/string.hpp"


namespace hexapod_kinematics
{
class Kinematics : public rclcpp::Node
{
public:
  Kinematics() : Node("") {};
  Kinematics(
    const std::vector<std::string>& feet_links,
    const std::string& base_link="base_link");
  void spinUntilInitialized();
  // TODO use more suitable type for goal_pos?
  int cartToJnt(
    const size_t leg_index, const KDL::JntArray& q_init,
    const KDL::Frame& T_base_goal, KDL::JntArray& q_out);

private:
  void robotDescriptionCallback(const std_msgs::msg::String& msg);
  void createSolvers();

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  KDL::Tree tree_;
  std::vector<std::string> feet_links_;
  std::string base_link_;
  // TODO use smart pointers to handle chains and solvers?
  std::vector<KDL::Chain> chains_;
  std::vector<KDL::ChainIkSolverPos_LMA> solvers_;
  bool solvers_set_ = false;
};
}   // namespace hexapod_kinematics

#endif  // HEXAPOD_KINEMATICS__KINEMATICS_HPP_
