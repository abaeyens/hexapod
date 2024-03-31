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
  const std::vector<std::string>& feet_links,
  const std::string& base_link)
: Node("hexapod_kinematics")
{
  feet_links_ = feet_links;
  base_link_ = base_link;
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "robot_description",
      rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&Kinematics::robotDescriptionCallback, this, _1));
}

void Kinematics::robotDescriptionCallback(const std_msgs::msg::String& msg)
{
  std::string urdf = msg.data;
  bool tree_was_not_empty = tree_.getNrOfSegments() > 0;
  bool success = kdl_parser::treeFromString(urdf, tree_);
  if (success) {
    RCLCPP_INFO(this->get_logger(),
      "IK: Constructed KDL tree from URDF with %d joints and %d segments.",
      tree_.getNrOfJoints(), tree_.getNrOfSegments());
  } else {
    RCLCPP_ERROR(this->get_logger(), "IK: Failed to construct KDL tree from URDF.");
    return;
  }
  if (tree_was_not_empty) {
    createSolvers();
    RCLCPP_INFO(this->get_logger(),
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
      RCLCPP_INFO(this->get_logger(),
        "IK: Extracted chain %d with %d joints and %d segments to link %s.",
        static_cast<int>(i), chains_[i].getNrOfJoints(),
        chains_[i].getNrOfSegments(), feet_links_[i].c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(),
        "IK: Failed to extract chain %d: %s => %s.",
        static_cast<int>(i), base_link_.c_str(), feet_links_[i].c_str());
      continue;
    }
    // Create IK solver
    solvers_.emplace_back(std::cref(chains_[i]), 1e-5, 100, 1e-15);
  }
  solvers_set_ = true;
}

void Kinematics::spinUntilInitialized()
{
  // TODO do this more elegantly (without polling)?
  while (!solvers_set_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "IK: Waiting until URDF received and solvers initialized.");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(this->get_node_base_interface());
  }
}

int Kinematics::cartToJnt(
  const size_t leg_index, const KDL::JntArray& q_init,
  const KDL::Frame& T_base_goal, KDL::JntArray& q_out)
{
  if (!solvers_set_) {
    RCLCPP_ERROR(this->get_logger(),
      "IK: Solvers not yet set, have to wait until initialized!");
    return -1;
  }
  return solvers_[leg_index].CartToJnt(q_init, T_base_goal, q_out);
}
}  // namespace hexapod_kinematics
