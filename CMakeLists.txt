cmake_minimum_required(VERSION 3.5)
project(visavis_vision)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV)


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
  message_generation
)

add_message_files(FILES
WallInfo.msg
WallInfoArray.msg
ObstacleInfo.msg
ObstacleInfoArray.msg
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)


find_package(Boost REQUIRED COMPONENTS system filesystem random date_time thread)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES 
  CATKIN_DEPENDS roslib roscpp pcl_msgs cv_bridge image_transport
  #  DEPENDS system_lib
  # DEPENDS boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(visavis_vision_node src/visavis_vision.cpp)
target_link_libraries( visavis_vision_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(combiner src/rgb_d_combiner.cpp)
target_link_libraries(combiner  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(coppelia_combiner src/coppelia_combiner.cpp)
target_link_libraries(coppelia_combiner  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(visavis_pcloud_stich src/pcloud_stitch.cc)
# target_compile_options(visavis_pcloud_stich PRIVATE ${OpenMP_FLAGS})
# target_link_libraries(visavis_pcloud_stich ${catkin_LIBRARIES} ${OpenMP_LIBS})
target_link_libraries(visavis_pcloud_stich ${catkin_LIBRARIES})
