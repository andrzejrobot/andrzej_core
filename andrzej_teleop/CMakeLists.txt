cmake_minimum_required(VERSION 2.8.3)
project(andrzej_teleop)

find_package(catkin REQUIRED COMPONENTS
  joy
  geometry_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS joy geometry_msgs roscpp
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(andrzej_joy src/andrzej_joy.cpp)

target_link_libraries(andrzej_joy
  ${catkin_LIBRARIES}
)
