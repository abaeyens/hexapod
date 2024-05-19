#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <limits>
#include <math.h>
#include <tgmath.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "actuator_msgs/msg/actuators.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"

#include "hexapod_kinematics/kinematics.hpp"
#include "hexapod_msgs/msg/vector3_array.hpp"
#include "hexapod_msgs/msg/gait_phase.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;

struct TrajectoryPose {
  double t;
  double x;
  double y;
  double z;
};

enum class GaitPhase {
  DOWN      = 0,
  RISING    = 1,
  UP        = 2,
  FALLING   = 3,
};

struct Leg {
  // neutral position, expressed in robot base_link
  KDL::Vector foot_center_position;
  // last foot position, expressed relative to neutral position
  KDL::Vector foot_relative_position;
  // last IK solution (to initialize the IK solver next time)
  KDL::JntArray joint_angles;
};

KDL::Vector get_goal_pos(KDL::Vector & v, const double rx, const double ry)
{
  const double alpha = atan2(v.y(), v.x());
  return KDL::Vector(cos(alpha) * rx, sin(alpha) * ry, 0);
}

class FollowVelocity : public rclcpp::Node
{
public:
  FollowVelocity()
  : Node("follow_velocity")
  {
    // Listen to velocity setpoint
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10, std::bind(&FollowVelocity::cmdVelCallback, this, _1));
    // Publisher for joint angle references
    publisher_ = this->create_publisher<actuator_msgs::msg::Actuators>(
      "actuators", 1);
    // Publisher for cartesian foot positions
    relative_foot_position_publisher_ =
      this->create_publisher<hexapod_msgs::msg::Vector3Array>(
        "relative_foot_positions", 1);
    // Publisher for gait phase
    gait_phase_left_publisher_ =
      this->create_publisher<hexapod_msgs::msg::GaitPhase>(
        "gait_phase_left", 1);
    gait_phase_right_publisher_ =
      this->create_publisher<hexapod_msgs::msg::GaitPhase>(
        "gait_phase_right", 1);

    // Initialize legs
    legs_.resize(nb_legs_);
    for (int i = 0; i < nb_legs_; i++) {
      legs_[i].foot_center_position.x(
        (i == 0 || i == 5 ? 0.18 : 0.00) + (i == 2 || i == 3 ? -0.18 : 0.00));
      legs_[i].foot_center_position.y(
        (i == 1 || i == 4 ? 0.27 : 0.20) * (i < 3 ? 1 : -1));
      legs_[i].foot_center_position.z(0);
      legs_[i].foot_relative_position.x(0);
      legs_[i].foot_relative_position.y(0);
      legs_[i].foot_relative_position.z((i % 2) ? pzmin_ : pzmax_);
      legs_[i].joint_angles = KDL::JntArray(nb_joints_per_leg_);
      // IK solver initialization
      legs_[i].joint_angles(0) = 0;
      legs_[i].joint_angles(1) = 0.1 * (i < 3 ? 1 : -1);
      legs_[i].joint_angles(1) = -1.5 * (i < 3 ? 1 : -1);
    }
    left_phase_ = GaitPhase::DOWN;
    right_phase_ = GaitPhase::UP;

    // Start listening to URDF, create KDL tree,
    // extract chains and construct a solver for each
    kinematics_ = std::make_shared<hexapod_kinematics::Kinematics>(
      std::vector<std::string>{"foot_0", "foot_1", "foot_2", "foot_3", "foot_4", "foot_5"});
    kinematics_->spinUntilInitialized();

    // Main timer callback, which moves the legs
    timer_ = this->create_wall_timer(
      20ms, std::bind(&FollowVelocity::timerCallback, this));
    last_start_ = this->now();

  }
private:
  inline GaitPhase getLegPhase(const int i)
  {
    // TODO generalize this to other numbers than six legs
    return i % 2 ? right_phase_ : left_phase_;
  }

  void cmdVelCallback(const geometry_msgs::msg::TwistStamped & msg)
  {
    cmd_vel_.vel.x(msg.twist.linear.x);
    cmd_vel_.vel.y(msg.twist.linear.y);
    cmd_vel_.vel.z(msg.twist.linear.z);
    cmd_vel_.rot.x(msg.twist.angular.x);
    cmd_vel_.rot.y(msg.twist.angular.y);
    cmd_vel_.rot.z(msg.twist.angular.z);
    cmd_vel_stamp_ = msg.header.stamp;
  }

  void timerCallback()
  {
    const rclcpp::Time now = this->now();
    // If velocity reference too old, don't do anything
    const double cmd_vel_oldness = (now - cmd_vel_stamp_).seconds();
    if (cmd_vel_oldness > cmd_vel_timeout_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Received cmd_vel timed out (%.2f > %.2f s).",
        cmd_vel_oldness, cmd_vel_timeout_);
      return;
    }

    // Don't change velocity if any leg in phase FALLING
    const bool falling =
      left_phase_ == GaitPhase::FALLING || right_phase_ == GaitPhase::FALLING;
    if (!falling) cmd_vel_smoothed_ = cmd_vel_;

    //const double dt = (now - last_start_).seconds();
    const double dt = 20e-3;
    const double duration_vertical = (pzmax_ - pzmin_) / vzmax_;

    // Velocity of each foot
    std::vector<KDL::Vector> foot_velocities(nb_legs_);
    for (int i = 0; i < nb_legs_; i++) {
      // TODO avoid writing out vector cross product
      const KDL::Twist & v = cmd_vel_smoothed_;
      const KDL::Vector r =
        legs_[i].foot_center_position + legs_[i].foot_relative_position;
      foot_velocities[i] = -KDL::Vector(
        v.vel.x() - v.rot.z() * r.y(), v.vel.y() + v.rot.z() * r.x(), 0);
    }

    // Get minimum duration until any of the supporting legs
    // transitions to RISING
    double min_duration_to_rising = std::numeric_limits<double>::infinity();
    for (int i = 0; i < nb_legs_; i++) {
      if (getLegPhase(i) != GaitPhase::DOWN)
        // Leg not supporting
        continue;
      const KDL::Vector & p = legs_[i].foot_relative_position;
      const KDL::Vector & fv = foot_velocities[i];
      // Duration to transition DOWN => RISING
      const double duration_to_rising_this_leg = std::min(
        (copysign(pxmax_, fv.x()) - p.x()) / fv.x(),
        (copysign(pymax_, fv.y()) - p.y()) / fv.y());
      if (duration_to_rising_this_leg < min_duration_to_rising)
        min_duration_to_rising = duration_to_rising_this_leg;
    }

    // State transitions
    if (left_phase_ == GaitPhase::DOWN &&
        right_phase_ == GaitPhase::FALLING) {
      // would go over limit
      // DOWN => RISING and FALLING => DOWN
      if (min_duration_to_rising - dt < 0) {
        left_phase_ = GaitPhase::RISING;
        right_phase_ = GaitPhase::DOWN;
      }
    } else if (right_phase_ == GaitPhase::DOWN &&
               left_phase_ == GaitPhase::FALLING) {
      if (min_duration_to_rising - dt < 0) {
        left_phase_ = GaitPhase::DOWN;
        right_phase_ = GaitPhase::RISING;
      }
    } else if (left_phase_ == GaitPhase::DOWN) {    
      // UP => FALLING
      if (min_duration_to_rising - dt < duration_vertical) {
        right_phase_ = GaitPhase::FALLING;
      }
    } else if (right_phase_ == GaitPhase::DOWN) {
      if (min_duration_to_rising - dt < duration_vertical) {
        left_phase_ = GaitPhase::FALLING;
      }
    }
    if (left_phase_ == GaitPhase::RISING &&
        legs_[0].foot_relative_position.z() + dt * vzmax_ > pzmax_) {
      // RISING => UP
      left_phase_ = GaitPhase::UP;
    } else if (right_phase_ == GaitPhase::RISING &&
               legs_[1].foot_relative_position.z() + dt * vzmax_ > pzmax_) {
      right_phase_ = GaitPhase::UP;
    }

    // Move legs, according to their state
    for (int i = 0; i < nb_legs_; i++) {
      KDL::Vector & p = legs_[i].foot_relative_position;
      switch (getLegPhase(i)) {
        case GaitPhase::DOWN:
          // In contact with ground, just integrate leg position
          p.z(pzmin_);
          p += foot_velocities[i] * dt;
          break;
        case GaitPhase::RISING:
          p.z(std::min(pzmax_, p.z() + vzmax_ * dt));
          break;
        case GaitPhase::UP:
        {
          p.z(pzmax_);
          const KDL::Vector goal_pos =
            -get_goal_pos(foot_velocities[i], pxmax_, pymax_);
          const double duration_to_down =
            min_duration_to_rising - duration_vertical;
          const KDL::Vector hor_foot_velocity(
            (goal_pos.x() - p.x()) / duration_to_down,
            (goal_pos.y() - p.y()) / duration_to_down,
            0);
          const double alpha = 1.2;
          p += alpha * hor_foot_velocity * dt;
          break;
        }
        case GaitPhase::FALLING:
          p.z(std::max(pzmin_, p.z() - vzmax_ * dt));
          if (p.z() < pzsync_) {
            // Foot close to ground, so already start moving it horizontally
            // such that it will be up to speed when it touches the ground.
            p += foot_velocities[i] * dt;
          }
          break;
      }
    }

    // Get joint angles (inverse kinematics)
    for (int i = 0; i < nb_legs_; i++) {
      const KDL::Vector foot_position =
        legs_[i].foot_center_position + legs_[i].foot_relative_position;
      kinematics_->cartToJnt(
        i, std::cref(legs_[i].joint_angles),
        KDL::Frame(foot_position),
        std::ref(legs_[i].joint_angles));
      kinematics_->foldAndClampJointAnglesToLimits(
        i, std::ref(legs_[i].joint_angles));
    }

    // Publish actuator setpoints
    actuator_msgs::msg::Actuators msg;
    msg.header.stamp = now;
    msg.position.resize(nb_legs_*nb_joints_per_leg_);
    for (int i = 0; i < nb_legs_; i++) {
      for (int j = 0; j < nb_joints_per_leg_; j++) {
        msg.position[i*nb_joints_per_leg_+j] = legs_[i].joint_angles(j);
      }
    }
    publisher_->publish(msg);
  
    // For debug, also publish relative foot cartesian positions
    hexapod_msgs::msg::Vector3Array foot_positions_msg;
    foot_positions_msg.header.stamp = now;
    foot_positions_msg.vectors.resize(nb_legs_);
    for (int i = 0; i < nb_legs_; i++) {
      foot_positions_msg.vectors[i].x = legs_[i].foot_relative_position.x();
      foot_positions_msg.vectors[i].y = legs_[i].foot_relative_position.y();
      foot_positions_msg.vectors[i].z = legs_[i].foot_relative_position.z();
    }
    relative_foot_position_publisher_->publish(foot_positions_msg);
    // and publish gait phases
    hexapod_msgs::msg::GaitPhase phase_left_msg;
    phase_left_msg.header.stamp = now;
    phase_left_msg.phase = static_cast<uint8_t>(left_phase_);
    gait_phase_left_publisher_->publish(phase_left_msg);
    hexapod_msgs::msg::GaitPhase phase_right_msg;
    phase_right_msg.header.stamp = now;
    phase_right_msg.phase = static_cast<uint8_t>(right_phase_);
    gait_phase_right_publisher_->publish(phase_right_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    cmd_vel_sub_;
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr publisher_;
  rclcpp::Publisher<hexapod_msgs::msg::Vector3Array>::SharedPtr
    relative_foot_position_publisher_;
  rclcpp::Publisher<hexapod_msgs::msg::GaitPhase>::SharedPtr
    gait_phase_left_publisher_;
  rclcpp::Publisher<hexapod_msgs::msg::GaitPhase>::SharedPtr
    gait_phase_right_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<hexapod_kinematics::Kinematics> kinematics_;
  rclcpp::Time last_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  std::vector<Leg> legs_;
  KDL::Twist cmd_vel_;
  KDL::Twist cmd_vel_smoothed_;
  rclcpp::Time cmd_vel_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  // Phases of the two front feet
  // alternating feet are in phase
  GaitPhase left_phase_;
  GaitPhase right_phase_;

  // Parameters
  static constexpr double cmd_vel_timeout_ = 0.3;

  // Hardware parameters
  // TODO derive those from properties.urdf.xacro
  static constexpr double pxmax_ = 0.045;
  static constexpr double pymax_ = 0.035;
  static constexpr double pzmin_ = -0.100;
  static constexpr double pzmax_ = -0.070;
  static constexpr double pzsync_ = -0.090;
  static constexpr double vxmax_ = 0.67 * 1/2;
  static constexpr double vymax_ = 0.34 * 1/2;
  static constexpr double vzmax_ = 0.34 * 1/2;
  // TODO support other values than 6
  static const int nb_legs_ = 6;
  static const int nb_joints_per_leg_ = 3;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowVelocity>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
