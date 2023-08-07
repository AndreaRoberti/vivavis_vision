cmake_minimum_required(VERSION 2.8.3)
project(vivavis_vision)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  pcl_conversions
  eigen_conversions
  pcl_ros
  tf_conversions
  cv_bridge
  image_transport
  pcl_msgs
  altair_msgs
  message_generation
)

add_message_files(FILES
WallInfo.msg
CloudArray.msg
PointArray.msg
PoseStampedArray.msg
Ellipsoid.msg
EllipsoidArray.msg
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)


find_package(Boost REQUIRED COMPONENTS system filesystem random date_time thread)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES 
  CATKIN_DEPENDS roslib roscpp pcl_msgs cv_bridge image_transport altair_msgs
  #  DEPENDS system_lib
  # DEPENDS boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(vivavis_vision_node src/vivavis_vision.cpp)
target_link_libraries( vivavis_vision_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(combiner src/rgb_d_combiner.cpp)
target_link_libraries(combiner  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
