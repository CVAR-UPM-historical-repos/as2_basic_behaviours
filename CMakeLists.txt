cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_basic_behaviours)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies

set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  px4_msgs
  as2_core
  as2_msgs
  px4_ros_com
  std_srvs
  as2_control_command_handlers
  trajectory_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

set(SOURCE_CPP_FILES 
  src/as2_basic_behaviour.cpp
  src/takeoff_behaviour.cpp
  src/gotowaypoint_behaviour.cpp
  src/follow_path_behaviour.cpp
  src/land_behaviour.cpp
)

# set(HEADER_HPP_FILES
#   include/as2_basic_behaviour.hpp
#   include/takeoff_behaviour.hpp
#   include/land_behaviour.hpp
#   include/gotowaypoint_behaviour.hpp
# )

include_directories(
  include
)

add_executable(land_behaviour_node src/land_behaviour_node.cpp ${SOURCE_CPP_FILES})
target_include_directories(land_behaviour_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(land_behaviour_node ${PROJECT_DEPENDENCIES})
 
add_executable(takeoff_behaviour_node src/takeoff_behaviour_node.cpp ${SOURCE_CPP_FILES})
target_include_directories(takeoff_behaviour_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(takeoff_behaviour_node ${PROJECT_DEPENDENCIES})


add_executable(gotowaypoint_behaviour_node src/gotowaypoint_behaviour_node.cpp ${SOURCE_CPP_FILES})
target_include_directories(gotowaypoint_behaviour_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(gotowaypoint_behaviour_node ${PROJECT_DEPENDENCIES})


add_executable(follow_path_behaviour_node src/follow_path_behaviour_node.cpp ${SOURCE_CPP_FILES})
target_include_directories(follow_path_behaviour_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(follow_path_behaviour_node ${PROJECT_DEPENDENCIES})


if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
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
  follow_path_behaviour_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
