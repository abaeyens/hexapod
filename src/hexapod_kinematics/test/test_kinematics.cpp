#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_msgs/msg/string.hpp"

#include "hexapod_kinematics/kinematics.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"


// Helper function: read URDF from file into String message
std_msgs::msg::String getRobotDescription(const std::string& path) {
  std::ifstream inFile(path);
  if(!inFile) throw std::runtime_error("Unable to open file: " + path);
  std::stringstream strStream;
  strStream << inFile.rdbuf();
  inFile.close();
  std_msgs::msg::String msg;
  msg.data = strStream.str();
  return msg;
}

TEST(hexapod_kinematics, parse_robot_description) {
    const std::string package_path =
        ament_index_cpp::get_package_share_directory("hexapod_kinematics");
    const std_msgs::msg::String robot_description =
        getRobotDescription(package_path + "/test/robot_description.urdf");

    rclcpp::init(0, nullptr);
    hexapod_kinematics::Kinematics kinematics(
        std::vector<std::string>{"foot"}, "base_link", false);
    kinematics.robotDescriptionCallback(robot_description);
    rclcpp::shutdown();

    const std::vector<KDL::Chain>& chains = kinematics.getChains();
    EXPECT_EQ(chains.size(), 1);
    EXPECT_EQ(chains[0].getNrOfJoints(), 2);
    EXPECT_EQ(chains[0].getNrOfSegments(), 3);

    const std::vector<std::vector<urdf::JointLimits>>&
        joint_limits = kinematics.getJointLimits();
    EXPECT_EQ(joint_limits.size(), 1);
    EXPECT_EQ(joint_limits[0].size(), 2);
    EXPECT_FLOAT_EQ(joint_limits[0][0].lower, -0.5);
    EXPECT_FLOAT_EQ(joint_limits[0][0].upper, 0.5);
}

TEST(hexapod_kinematics, run_inverse_kinematics) {
    const std::string package_path =
        ament_index_cpp::get_package_share_directory("hexapod_kinematics");
    const std_msgs::msg::String robot_description =
        getRobotDescription(package_path + "/test/robot_description.urdf");

    rclcpp::init(0, nullptr);
    hexapod_kinematics::Kinematics kinematics(
        std::vector<std::string>{"foot"}, "base_link", false);
    kinematics.robotDescriptionCallback(robot_description);
    KDL::JntArray q_init(2);
    q_init(0) = -0.1;
    q_init(1) = 0.2;
    const KDL::Frame p_in(KDL::Vector(0.3, 0.0, -0.6));
    KDL::JntArray q_out(2);
    const int result = kinematics.cartToJnt(0, q_init, p_in, q_out);
    const bool clamped = kinematics.foldAndClampJointAnglesToLimits(0, q_out);
    rclcpp::shutdown();
    /*
    std::cout << "solver return: " << result << std::endl;
    std::cout << "clamped:       " << clamped << std::endl;
    std::cout << "joint angles [deg]: " << q_out(0)*180/M_PI << ", "
                                        << q_out(1)*180/M_PI << std::endl;
    */
    EXPECT_EQ(result, -101);
    EXPECT_EQ(clamped, true);
    EXPECT_NEAR(q_out(0)*180/M_PI, -28.6479, 1e-1);
    EXPECT_NEAR(q_out(1)*180/M_PI, -28.6479, 1e-1);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
