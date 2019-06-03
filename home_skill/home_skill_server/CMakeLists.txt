cmake_minimum_required(VERSION 2.8.3)
project(home_skill_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  home_skill_msgs
  std_msgs
  trac_ik_lib
  control_msgs
)

file(GLOB PROGRAM_HEADERS RELATIVE ${PROJECT_SOURCE_DIR} "include/${PROJECT_NAME}/*.h")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  trac_ik_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

 add_executable(${PROJECT_NAME} src/home_skill_server.cpp src/main.cpp src/ur5.cpp ${PROGRAM_HEADERS})
 target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++1y -g -O3")
