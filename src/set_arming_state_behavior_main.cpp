#include "as2_basic_behaviors/set_arming_state_behavior.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetArmingStateBehavior>());
  rclcpp::shutdown();
  return 0;
}
