cmake_minimum_required(VERSION 2.8.3)
project(strttc_daheng)
add_definitions(
    -std=c++11
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  dv_ros_msgs
  image_transport
  camera_info_manager
  sensor_msgs
)


find_package(OpenCV)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(strttc_daheng src/daheng.cpp)
target_link_libraries(strttc_daheng ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
pthread
gxiapi
)



install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
