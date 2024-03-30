#include "hexapod_kinematics/kinematics.hpp"

#include "rclcpp/qos.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl_parser/kdl_parser.hpp"

namespace hexapod_kinematics
{
using std::placeholders::_1;

Kinematics::Kinematics(
  const rclcpp::Node::SharedPtr& nh,
  const std::vector<std::string>& feet_links,
  const std::string& base_link)
{
  nh_ = nh;
  feet_links_ = feet_links;
  base_link_ = base_link;
  subscription_ = nh_->create_subscription<std_msgs::msg::String>(
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
    RCLCPP_INFO(nh_->get_logger(),
      "IK: Constructed KDL tree from URDF with %d joints and %d segments.",
      tree_.getNrOfJoints(), tree_.getNrOfSegments());
  } else {
    RCLCPP_ERROR(nh_->get_logger(), "IK: Failed to construct KDL tree from URDF.");
    return;
  }
  if (tree_was_not_empty) {
    createSolvers();
    RCLCPP_INFO(nh_->get_logger(),
      "IK: Received a new URDF, processed it and rebuilt the solvers.");
  }
  createSolvers();
}

void Kinematics::createSolvers()
{
  chains_.clear();
  chains_.resize(feet_links_.size());
  solvers_.clear();
  solvers_.reserve(feet_links_.size());
  for (size_t i = 0; i < feet_links_.size(); i++) {
    // Extract chain of interest from tree
    bool success = tree_.getChain(base_link_, feet_links_[i], std::ref(chains_[i]));
    if (success) {
      RCLCPP_INFO(nh_->get_logger(),
        "IK: Extracted chain %d with %d joints and %d segments to link %s.",
        static_cast<int>(i), chains_[i].getNrOfJoints(),
        chains_[i].getNrOfSegments(), feet_links_[i].c_str());
    } else {
      RCLCPP_ERROR(nh_->get_logger(),
        "IK: Failed to extract chain %d: %s => %s.",
        static_cast<int>(i), base_link_.c_str(), feet_links_[i].c_str());
      continue;
    }
    // Create IK solver
    solvers_.emplace_back(std::cref(chains_[i]), 1e-5, 100, 1e-15);
  }
  solvers_set_ = true;
}

void Kinematics::wait_until_initialized()
{
  // TODO do this more elegantly (without polling)?
  while (!solvers_set_) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(nh_);
  }
}

int Kinematics::cartToJnt(
  const size_t leg_index, const KDL::JntArray& q_init,
  const KDL::Frame& T_base_goal, KDL::JntArray& q_out)
{
  if (!solvers_set_) {
    RCLCPP_ERROR(nh_->get_logger(),
      "Solvers not yet set, first call method registerFeet!");
    return -1;
  }
  return solvers_[leg_index].CartToJnt(q_init, T_base_goal, q_out);
}
}  // namespace hexapod_kinematics
