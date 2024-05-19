#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "actuator_msgs/msg/actuators.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"

#include "hexapod_kinematics/kinematics.hpp"

using namespace std::chrono_literals;

struct TrajectoryPose {
  double t;
  double x;
  double y;
  double z;
};

void interpolate_trajectory(
  const std::vector<TrajectoryPose> & trajectory, const double time, TrajectoryPose & p) {
  // TODO update this to use faster bisection method
  if (trajectory.size() == 1) p = trajectory[0];
  for (size_t i = 1; i < trajectory.size(); i++)  {
    if (trajectory[i].t > time) {
      const double alpha = (time - trajectory[i-1].t) / (trajectory[i].t - trajectory[i-1].t);
      p.t = time;
      p.x = trajectory[i-1].x * (1-alpha) + trajectory[i].x * alpha;
      p.y = trajectory[i-1].y * (1-alpha) + trajectory[i].y * alpha;
      p.z = trajectory[i-1].z * (1-alpha) + trajectory[i].z * alpha;
      return;
    }
  }
  p.t = trajectory.back().t;
  p.x = trajectory.back().x;
  p.y = trajectory.back().y;
  p.z = trajectory.back().z;
}


class WalkForward : public rclcpp::Node
{
public:
  WalkForward()
  : Node("walk_forward")
  {
    // Load trajectory from given filepath
    this->declare_parameter("trajectory_filepath", "");
    std::string trajectory_filepath =
        this->get_parameter("trajectory_filepath").as_string();
    load_trajectory_from_file(trajectory_filepath);

    // Callbacks
    publisher_ = this->create_publisher<actuator_msgs::msg::Actuators>(
      "actuators", 1);
    timer_ = this->create_timer(
      5ms, std::bind(&WalkForward::timerCallback, this));
    last_start_ = this->now();

    // Initialize joint angles (solver initialization)
    joint_angles_.resize(6);
    for (int i = 0; i < nb_legs_; i++) {
      joint_angles_[i] = KDL::JntArray(nb_joints_per_leg_);
      joint_angles_[i](0) = 0;
      joint_angles_[i](1) = 0.1 * (i < 3 ? 1 : -1);
      joint_angles_[i](2) = -1.5 * (i < 3 ? 1 : -1);
    }
  
    // Start listening to URDF, create KDL tree,
    // extract chains and construct a solver for each
    kinematics_ = std::make_shared<hexapod_kinematics::Kinematics>(
      std::vector<std::string>{"foot_0", "foot_1", "foot_2", "foot_3", "foot_4", "foot_5"});
    kinematics_->spinUntilInitialized();
  }
private:
  void timerCallback()
  {
    // Get relative time of where we are in the foot motion
    const double trajectory_duration = leg_trajectory_.back().t;
    const double period = trajectory_duration * 1.0;
    rclcpp::Time now = this->now();
    if (now - last_start_ > rclcpp::Duration::from_seconds(period))
      last_start_ += rclcpp::Duration::from_seconds(period);
    const double rt_even = (now - last_start_).seconds() * trajectory_duration / period;
    const double rt_odd = fmod(rt_even + trajectory_duration/2, trajectory_duration);
    // Read out the poses for even and odd feet
    TrajectoryPose p_even, p_odd;
    interpolate_trajectory(leg_trajectory_, rt_even, p_even);
    interpolate_trajectory(leg_trajectory_, rt_odd, p_odd);
    // Transform to feet points
    std::vector<KDL::Vector> foot_positions(nb_legs_);
    for (int i = 0; i < nb_legs_; i++) {
      TrajectoryPose p = i % 2 ? p_odd : p_even;
      foot_positions[i].x(p.x + (i == 0 || i == 5 ? 0.18 : 0.00) + (i == 2 || i == 3 ? -0.18 : 0.00));
      foot_positions[i].y((p.y + (i == 1 || i == 4 ? 0.27 : 0.20)) * (i < 3 ? 1 : -1));
      foot_positions[i].z(p.z -= 0.10);
    }
    // Get joint angles (inverse kinematics)
    for (int i = 0; i < nb_legs_; i++) {
      kinematics_->cartToJnt(
        i, std::cref(joint_angles_[i]), KDL::Frame(foot_positions[i]),
        std::ref(joint_angles_[i]));
      kinematics_->foldAndClampJointAnglesToLimits(
        i, std::ref(joint_angles_[i]));
    }
    // Publish actuator setpoints
    actuator_msgs::msg::Actuators msg;
    msg.header.stamp = this->now();
    msg.position.resize(nb_legs_*nb_joints_per_leg_);
    for (int i = 0; i < nb_legs_; i++) {
      for (int j = 0; j < nb_joints_per_leg_; j++) {
        msg.position[i*nb_joints_per_leg_+j] = joint_angles_[i](j);
      }
    }
    publisher_->publish(msg);
  }

  void load_trajectory_from_file(const std::string & filename) {
    if (filename.empty())
     throw std::runtime_error("Given filename empty.");
    std::ifstream file(filename);
    if (!file.is_open())
      throw std::runtime_error("Could not open file " + filename + ".");
    if (!file.good())
      throw std::runtime_error("File is not good: " + filename + ".");
    leg_trajectory_ = {};
    std::string line;
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::istringstream iss(line);
      TrajectoryPose pose;
      char comma;
      if (iss >> pose.t >> comma >> pose.x >> comma >> pose.y >> comma >> pose.z) {
        leg_trajectory_.push_back(pose);
      } else {
        std::cerr << "Error parsing line: " << line << std::endl;
      }
    }
    // Let trajectory start exactly at t = 0
    for (TrajectoryPose & p: leg_trajectory_) p.t -= leg_trajectory_[0].t;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr publisher_;
  std::shared_ptr<hexapod_kinematics::Kinematics> kinematics_;
  rclcpp::Time last_start_;
  std::vector<KDL::JntArray> joint_angles_;
  std::vector<TrajectoryPose> leg_trajectory_;

  static const int nb_legs_ = 6;
  static const int nb_joints_per_leg_ = 3;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WalkForward>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
