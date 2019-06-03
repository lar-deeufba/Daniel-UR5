cmake_minimum_required(VERSION 2.8.3)
 project(place_avg_skill_client)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS roscpp rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
