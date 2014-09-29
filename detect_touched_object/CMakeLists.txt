cmake_minimum_required(VERSION 2.8.3)
project(detect_touched_object)

find_package(catkin REQUIRED COMPONENTS
  image_geometry
  image_transport
  cv_bridge
  tf
  image_view
  jsk_pcl_ros
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(extract_object_image src/extract_object_image.cpp)
target_link_libraries(extract_object_image ${catkin_LIBRARIES})
add_dependencies(extract_object_image ${catkin_EXPORTED_TARGETS} ${OpenCV_LIBRARIES})
