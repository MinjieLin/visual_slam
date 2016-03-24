cmake_minimum_required(VERSION 2.8.3)

project(visual_features_extractor)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
  cmake_modules
  dynamic_reconfigure 
  message_generation
  message_filters
  visual_slam_msgs
)

SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++11)

#find_package(OpenCV 2.4.3 REQUIRED)
#SET(CMAKE_LIBRARY_PATH /home/robotis/opencv-2.4.12/build/lib/ ${CMAKE_LIBRARY_PATH})
#find_library(OpenCv 2.4 REQUIRED)
#message(STATUS ${OpenCv_LIBRARIES})
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals thread)

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(
  cfg/FeatureDetector.cfg
)

catkin_package()

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(feature_detector src/feature_detector.cpp)

set(CUSTOM_CV /home/robotis/opencv-2.4.12/build/lib/libopencv_features2d.so)
if(EXISTS ${CUSTOM_CV})
  target_link_libraries(feature_detector ${CUSTOM_CV} ${catkin_LIBRARIES}) 
else()
  target_link_libraries(feature_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endif()
#target_link_libraries(feature_detector ${catkin_LIBRARIES})
#target_link_libraries(feature_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(feature_detector ${PROJECT_NAME}_generate_messages_cpp)
