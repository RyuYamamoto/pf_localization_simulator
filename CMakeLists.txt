cmake_minimum_required(VERSION 3.0.2)
project(particle_filter_localization)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf2
  tf2_ros
  nav_sim
)


find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES particle_filter_localization
#  CATKIN_DEPENDS geometry_msgs roscpp tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(particle_filter_localization_node src/particle_filter_localization.cpp src/particle_filter_localization_node.cpp)
target_link_libraries(particle_filter_localization_node ${catkin_LIBRARIES} yaml-cpp)
