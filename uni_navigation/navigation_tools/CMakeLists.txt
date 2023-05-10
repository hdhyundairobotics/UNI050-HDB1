cmake_minimum_required(VERSION 3.0.2)
project(navigation_tools)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navigation_tools
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(
  DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


