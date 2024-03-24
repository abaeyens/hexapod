#include "hexapod_ik/leg_inverse.hpp"

#include "kdl/chainiksolverpos_lma.hpp"
#include "rclcpp/qos.hpp"

namespace hexapod_ik
{
using std::placeholders::_1;

LegInverse::LegInverse() : Node("leg_inverse_kinematics_solver")
{
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "robot_description",
      rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&LegInverse::robot_description_callback, this, _1));
}

void LegInverse::robot_description_callback(const std_msgs::msg::String& msg)
{
  // Construct KDL tree from URDF
  urdf_ = msg.data;
  bool success = kdl_parser::treeFromString(urdf_, tree_);
  RCLCPP_INFO(this->get_logger(), "Constructed KDL tree from URDF: %i",
              success);
  std::cout << "nb joints:    " << tree_.getNrOfJoints() << std::endl;
  std::cout << "nb segments:  " << tree_.getNrOfSegments() << std::endl;
  std::cout << "root segment: " << tree_.getRootSegment()->first << std::endl;
  const KDL::SegmentMap& segments = tree_.getSegments();
  for (const auto& seg : segments) {
    std::cout << "  got segment " << seg.first << std::endl;
  }
  // Get kinematic chain of first leg
  KDL::Chain chain;
  tree_.getChain("base_link", "foot_0", chain);
  std::cout << "nb_joints:    " << chain.getNrOfJoints() << std::endl;

  // Create IK solver
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver =
      std::make_unique<KDL::ChainIkSolverPos_LMA>(chain, 1e-5, 100, 1e-15);

  // Run IK solver
  KDL::JntArray q_init(3);
  q_init(0) = 0;
  q_init(1) = 0.1;
  q_init(2) = -0.2;
  KDL::JntArray q_out(3);
  KDL::Frame p_in(KDL::Vector(0.16, 0.24, -0.03));
  int result = solver->CartToJnt(q_init, p_in, q_out);
  std::cout << "solver return: " << result << std::endl;
  std::cout << "joint angles: " << q_out(0) * 180 / M_PI << ", "
            << q_out(1) * 180 / M_PI << ", " << q_out(2) * 180 / M_PI
            << std::endl;
}
}  // namespace hexapod_ik

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hexapod_ik::LegInverse>());
  rclcpp::shutdown();
  return 0;
}
