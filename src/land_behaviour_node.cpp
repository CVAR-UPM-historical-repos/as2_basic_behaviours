// "Copyright [year] <Copyright Owner>"

#include "land_behaviour.hpp"

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  std::cout << "Starting Land Behaviour" << std::endl;
  rclcpp::spin(std::make_shared<LandBehaviour>());
  rclcpp::shutdown();
  return 0;
}
