#include "hexapod_kinematics/kinematics.hpp"

#include "rclcpp/rclcpp.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"

class ExampleNode : public rclcpp::Node
{
public:
  ExampleNode()
  : Node("example_node_kinematics_usage")
  {
    // Start listening to URDF and create KDL tree
    hexapod_kinematics::Kinematics kinematics(shared_from_this());

    // Extract kinematic chains and construct a solver for each
    kinematics.registerFeet(std::vector<std::string>{"foot_0", "foot_1", "foot_2"});

    // Run the solver once
    KDL::JntArray q_init(3);
    q_init(0) = 0;
    q_init(1) = 0.1;
    q_init(2) = -0.2;
    KDL::Frame p_in(KDL::Vector(0.16, 0.24, -0.03));
    KDL::JntArray q_out(3);
    int result = kinematics.cartToJnt("foot_0", q_init, p_in, q_out);
    std::cout << "solver return: " << result << std::endl;
    std::cout << "joint angles [deg]: " << q_out(0)*180/M_PI << ", "
                                        << q_out(1)*180/M_PI << ", "
                                        << q_out(2)*180/M_PI << std::endl;
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExampleNode>());
  rclcpp::shutdown();
  return 0;
}
