cmake_minimum_required(VERSION 3.0.2)
project(intel_realsense_ball_detecting_and_ranging)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES intel_realsense_ball_detecting_and_ranging
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(
  ${PROJECT_NAME} 
    src/intel_realsense_ball_detection_and_ranging.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES
#  ${PROJECT_NAME}
#    launch/.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)