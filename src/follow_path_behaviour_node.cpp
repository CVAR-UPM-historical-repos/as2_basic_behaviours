// "Copyright [year] <Copyright Owner>"

#include "as2_basic_behaviour.hpp"
#include "follow_path_behaviour.hpp"


int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  std::cout << "Starting Go To Waypoint Behaviour" << std::endl;
  rclcpp::spin(std::make_shared<FollowPathBehaviour>());
  rclcpp::shutdown();
  return 0;
}
