cmake_minimum_required(VERSION 2.8.3)
project(andrzej_teleop)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    joy
    geometry_msgs
    controller_manager_msgs
    roscpp
    )

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    joy
    geometry_msgs
    roscpp
    controller_manager_msgs
    DEPENDS
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_library(andrzej_joint_manager
    src/JointManager.cpp
    src/Joint.cpp src/Joint.h)

add_executable(andrzej_joy src/andrzej_joy.cpp)

target_link_libraries(andrzej_joy
    andrzej_joint_manager
    ${catkin_LIBRARIES}
    )

add_executable(andrzej_key src/andrzej_key.cpp)

target_link_libraries(andrzej_key
    andrzej_joint_manager
    ${catkin_LIBRARIES}
    )