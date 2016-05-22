cmake_minimum_required(VERSION 2.8.3)
project(andrzej_control)

set(CMAKE_CXX_FLAGS "-std=c++1y ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    controller_manager
    hardware_interface
    sensor_msgs
    urdf
    trajectory_msgs
    )

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES pca9685
    CATKIN_DEPENDS
    roscpp
    urdf
    controller_manager
    hardware_interface
    sensor_msgs
    trajectory_msgs
)

# include_directories(include)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(pca9685 src/PCA9685.cpp src/I2C.cpp)
target_link_libraries(pca9685 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(servo_test src/servo_test.cpp)
target_link_libraries(servo_test
    pca9685
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )

add_executable(andrzej_hardware_interface src/andrzej_hardware_interface.cpp src/joint_hardware_interface.cpp)
target_link_libraries(andrzej_hardware_interface
    pca9685
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )

add_dependencies(andrzej_hardware_interface ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS andrzej_control andrzej_control_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_andrzej_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
