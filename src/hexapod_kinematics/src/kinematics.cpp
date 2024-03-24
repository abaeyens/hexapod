#include "hexapod_kinematics/kinematics.hpp"

#include "rclcpp/qos.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl_parser/kdl_parser.hpp"

namespace hexapod_kinematics
{
using std::placeholders::_1;

Kinematics::Kinematics(const rclcpp::Node::SharedPtr& node)
{
  node_ = node;
  subscription_ = node_->create_subscription<std_msgs::msg::String>(
      "robot_description",
      rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&Kinematics::robot_description_callback, this, _1));
}

void Kinematics::robot_description_callback(const std_msgs::msg::String& msg)
{
  std::string urdf = msg.data;
  bool tree_was_not_empty = tree_.getNrOfSegments() > 0;
  bool success = kdl_parser::treeFromString(urdf, tree_);
  if (success) {
    RCLCPP_INFO(
      node_->get_logger(),
      "IK: Constructed KDL tree from URDF with %d joints and %d segments.",
      tree_.getNrOfJoints(), tree_.getNrOfSegments());
  } else {
    RCLCPP_ERROR(node_->get_logger(), "IK: Failed to construct KDL tree from URDF.");
    return;
  }
  if (tree_was_not_empty) {
    createSolvers();
    RCLCPP_INFO(
      node_->get_logger(),
      "IK: Received a new URDF, processed it and rebuilt the solvers.");
  }
}

void Kinematics::registerFeet(
  const std::vector<std::string>& feet_links, const std::string& base_link)
{
  feet_links_ = feet_links;
  base_link_ = base_link;
  createSolvers();
}

void Kinematics::createSolvers()
{
  solver_map_.clear();
  for (std::string link: feet_links_) {
    // Extract chain of interest from tree
    KDL::Chain chain;
    bool success = tree_.getChain(base_link_, link, chain);
    if (success) {
      RCLCPP_INFO(
        node_->get_logger(),
        "IK: Extracted chain with %d joints and %d segments to link %s.",
        chain.getNrOfJoints(), chain.getNrOfSegments(), link.c_str());
    } else {
      RCLCPP_ERROR(
        node_->get_logger(), "IK: Failed to extract chain %s => %s.",
        base_link_.c_str(), link.c_str());
        return;
    }
    // Create IK solver
    KDL::ChainIkSolverPos_LMA solver(chain, 1e-5, 100, 1e-15);
    solver_map_.insert({link, solver});
  }
}

int Kinematics::cartToJnt(
  const std::string& feet_link, const KDL::JntArray& q_init,
  const KDL::Frame& T_base_goal, KDL::JntArray& q_out)
{
  return solver_map_.at(feet_link).CartToJnt(q_init, T_base_goal, q_out);
}
}  // namespace hexapod_kinematics
