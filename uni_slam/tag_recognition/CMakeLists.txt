cmake_minimum_required(VERSION 3.0.2)
project(tag_recognition)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
  tf2_msgs
  tf
  tf_conversions
  nav_msgs
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

## Generate messages in the 'msg' folder
add_message_files(
  FILES 
  RobotError.msg
  )


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(tag_recognition src/tag_recognition.cpp)
add_dependencies(tag_recognition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tag_recognition ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})


#############
## Install ##
#############
install(
  TARGETS tag_recognition
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install(TARGETS cliff_detect
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY config/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
#   FILES_MATCHING
#   PATTERN "*.yaml"
#   PATTERN "*.txt"
#   #PATTERN "*.conf"
#   PATTERN ".svn" EXCLUDE
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   FILES_MATCHING PATTERN "*.launch"
#   PATTERN ".svn" EXCLUDE)

