#include <cstdio>
#include <vector>
#include <chrono>
#include <cmath>

#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TestROS2Bridge : public rclcpp::Node
{
  public:

  TestROS2Bridge() : Node("TestROS2Bridge")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_command",10);
    timer_ = this->create_wall_timer(0.05s, std::bind(&TestROS2Bridge::timer_callback, this));

    joint_state_.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint","finger_joint"};
    
    num_joints_ = joint_state_.name.size();
    
    //initialize joint positions
    joint_state_.position.resize(num_joints_, 0.0);

    default_joints_ = {0.0, 0.0, -0.0, -0.0, -0.0, 0.0, 0.1};

    max_joints_.resize(num_joints_);
    min_joints_.resize(num_joints_);

    for (size_t i = 0; i < num_joints_; ++i) {
      if(i == 0 || i == 6){
        max_joints_[i] = default_joints_[i] + 0.5;
        min_joints_[i] = default_joints_[i] - 0.5;
      }
    }

    start_time_ = this->now().seconds();

  }

  void timer_callback() {
    joint_state_.header.stamp = this->get_clock()->now();

    double current_time = this->now().seconds() - start_time_;

    for (size_t i = 0; i < num_joints_; ++i) {
      joint_state_.position[i] = std::sin(current_time) * 0.5 * (max_joints_[i] - min_joints_[i]) + default_joints_[i];
    }

    publisher_->publish(joint_state_);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::JointState joint_state_;

  std::vector<double> default_joints_;
  std::vector<double> max_joints_;
  std::vector<double> min_joints_;

  size_t num_joints_;
  double start_time_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world arm_move1 package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestROS2Bridge>());
  rclcpp::shutdown();

  return 0;
}
