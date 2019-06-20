cmake_minimum_required(VERSION 2.8.3)
project(slam_exploration)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
  tf
  mav_msgs
)

find_package(Eigen REQUIRED)

#generate_dynamic_reconfigure_options(cfg/Controller.cfg)

catkin_package(
  INCLUDE_DIRS ${Eigen_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs mav_msgs
  DEPENDS Eigen
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(waypoint_generator_node src/waypoint_generator.cpp)
add_dependencies(waypoint_generator_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_generator_node ${catkin_LIBRARIES} yaml-cpp)
