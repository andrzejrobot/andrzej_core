cmake_minimum_required(VERSION 2.8.3)
project(andrzej_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
