#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "actuator_msgs/msg/actuators.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"

#include "hexapod_kinematics/kinematics.hpp"

using namespace std::chrono_literals;

struct Point {
  double t;
  double x;
  double y;
  double z;
};
static const std::vector<Point> leg_trajectory = {
  {0,  0, 0, 0},
  {2, -1, 0, 0},
  {3, -1, 1, 1},
  {5,  1, 1, 1},
  {6,  1, 0, 0},
  {8,  0, 0, 0},
};

static const double period = 3.0;
static const double amplitude_x = 0.05;
static const double amplitude_y = 0;
static const double amplitude_z = 0.04;
static const double offset_z = -0.08;

void interpolate_trajectory(
  const std::vector<Point> & trajectory, const double time, Point & p) {
  if (trajectory.size() == 1) p = trajectory[0];
  for (size_t i = 1; i < trajectory.size(); i++)  {
    if (trajectory[i].t > time) {
      const double alpha = (time - trajectory[i-1].t) / (trajectory[i].t - trajectory[i-1].t);
      p.t = time;
      p.x = trajectory[i-1].x * (1-alpha) + trajectory[i].x * alpha;
      p.y = trajectory[i-1].y * (1-alpha) + trajectory[i].y * alpha;
      p.z = trajectory[i-1].z * (1-alpha) + trajectory[i].z * alpha;
      break;
    }
  }
}


class WalkForward : public rclcpp::Node
{
public:
  WalkForward()
  : Node("walk_forward")
  {
    publisher_ = this->create_publisher<actuator_msgs::msg::Actuators>(
      "actuators", 1);
    timer_ = this->create_wall_timer(
      20ms, std::bind(&WalkForward::timerCallback, this));
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
    rclcpp::Time now = this->now();
    if (now - last_start_ > rclcpp::Duration(period, 0))
      last_start_ += rclcpp::Duration(period, 0);
    const double trajectory_duration = leg_trajectory.back().t;
    const double rt_even = (now - last_start_).seconds() / period * trajectory_duration;
    const double rt_odd = fmod(rt_even + trajectory_duration/2, trajectory_duration);
    // Read out the poses for even and odd feet
    Point p_even, p_odd;
    interpolate_trajectory(leg_trajectory, rt_even, p_even);
    interpolate_trajectory(leg_trajectory, rt_odd, p_odd);
    p_even.x *= amplitude_x;
    p_even.y *= amplitude_y;
    p_even.z = p_even.z * amplitude_z + offset_z;
    p_odd.x *= amplitude_x;
    p_odd.y *= amplitude_y;
    p_odd.z = p_odd.z * amplitude_z + offset_z;
    // Transform to feet points
    std::vector<KDL::Vector> foot_positions(nb_legs_);
    for (int i = 0; i < nb_legs_; i++) {
      Point p = i % 2 ? p_odd : p_even;
      foot_positions[i].x(p.x + (i == 0 || i == 5 ? 0.18 : 0.00) + (i == 2 || i == 3 ? -0.18 : 0.00));
      foot_positions[i].y((p.y + (i == 1 || i == 4 ? 0.27 : 0.20)) * (i < 3 ? 1 : -1));
      foot_positions[i].z(p.z);
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
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr publisher_;
  std::shared_ptr<hexapod_kinematics::Kinematics> kinematics_;
  rclcpp::Time last_start_;
  std::vector<KDL::JntArray> joint_angles_;

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
