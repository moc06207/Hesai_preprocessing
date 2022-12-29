cmake_minimum_required(VERSION 3.0.2)
project(hesai)


find_package(catkin REQUIRED COMPONENTS
   roscpp
   pcl_conversions
   pcl_ros
   std_msgs
   message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_generation roscpp std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_executable(Point_parshing_1 src/hesai_parsh.cpp)
target_link_libraries(Point_parshing_1 ${catkin_LIBRARIES})

add_executable(Velodyne_Point_parshing_1 src/velodyne_parsh.cpp)
target_link_libraries(Velodyne_Point_parshing_1 ${catkin_LIBRARIES})

add_executable(TF_publisher src/frame_src/tf_broadcaster.cpp)
target_link_libraries(TF_publisher ${catkin_LIBRARIES})