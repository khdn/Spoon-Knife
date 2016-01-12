cmake_minimum_required(VERSION 2.8.3)
project(people_and_car_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(
	people_detector 
	src/people_detector.cpp
)


target_link_libraries(
	people_detector 
	${catkin_LIBRARIES}
	${OpenCV_INCLUDE_DIRS}
)
