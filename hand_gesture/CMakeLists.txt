cmake_minimum_required(VERSION 2.8.3)
project(hand_gesture)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  roscpp
  std_msgs
  image_transport
  pcl_ros
  tf
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(elbow_hand_line src/elbow_hand_line.cpp)
target_link_libraries(elbow_hand_line ${catkin_LIBRARIES})
add_dependencies(elbow_hand_line ${catkin_EXPORTED_TARGETS} ${OpenCV_LIBRARIES})
