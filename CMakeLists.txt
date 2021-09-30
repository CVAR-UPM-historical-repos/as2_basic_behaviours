cmake_minimum_required(VERSION 3.5)
project(pixhawk_test)
set(PROJECT_NAME as2_basic_behaviours)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(aerostack2_core REQUIRED)
find_package(aerostack2_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(std_srvs REQUIRED)
find_package(as2_control_command_handlers REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(SOURCE_CPP_FILES 
  src/as2_basic_behaviour.cpp
  src/takeoff_behaviour.cpp
  )

set(HEADER_HPP_FILES
  include/as2_basic_behaviour.hpp
  include/takeoff_behaviour.hpp
  include/land_behaviour.hpp
  include/gotowaypoint_behaviour.hpp
)

include_directories(
  include
)
set(DEPENDENCIES_LIST
rclcpp 
sensor_msgs 
std_msgs
std_srvs
nav_msgs
px4_msgs
px4_ros_com
aerostack2_core
aerostack2_msgs
geometry_msgs
rclcpp_action
as2_control_command_handlers
)

add_executable(land_behaviour_node src/land_behaviour_node.cpp ${SOURCE_CPP_FILES} ${HEADER_HPP_FILES})
ament_target_dependencies(land_behaviour_node ${DEPENDENCIES_LIST})
 
add_executable(takeoff_behaviour_node src/takeoff_behaviour_node.cpp ${SOURCE_CPP_FILES} ${HEADER_HPP_FILES})
ament_target_dependencies(takeoff_behaviour_node ${DEPENDENCIES_LIST})

add_executable(gotowaypoint_behaviour_node src/gotowaypoint_behaviour_node.cpp ${SOURCE_CPP_FILES} ${HEADER_HPP_FILES})
ament_target_dependencies(gotowaypoint_behaviour_node ${DEPENDENCIES_LIST})


if(BUILD_TESTING)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cpplint(src/ include/)
  ament_cppcheck(src/ include/)
  ament_clang_format(src/ include/)

endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  land_behaviour_node
  takeoff_behaviour_node
  gotowaypoint_behaviour_node
  DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
