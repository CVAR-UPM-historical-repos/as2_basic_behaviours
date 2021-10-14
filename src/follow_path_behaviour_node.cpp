// "Copyright [year] <Copyright Owner>"

#include "as2_basic_behaviour.hpp"
#include "follow_path_behaviour.hpp"


int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("as2_basic_behaviours"), "Starting FOLLOW PATH BEHAVIOUR");
  rclcpp::spin(std::make_shared<FollowPathBehaviour>());
  rclcpp::shutdown();
  return 0;
}
