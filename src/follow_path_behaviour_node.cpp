// "Copyright [year] <Copyright Owner>"

#include "as2_basic_behaviour.hpp"
#include "follow_path_behaviour.hpp"
#include "as2_core/core_functions.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("as2_basic_behaviours"), "Starting FOLLOW PATH BEHAVIOUR");

  auto node = std::make_shared<FollowPathBehaviour>();
  node->preset_loop_frequency(30);
  as2::spinLoop(node);

  rclcpp::shutdown();
  return 0;
}
