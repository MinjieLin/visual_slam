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
)


find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals thread)

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(
  cfg/FeatureDetector.cfg
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   KeyPoint.msg
   Frame.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package()

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(feature_detector src/feature_detector.cpp)
target_link_libraries(feature_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(feature_detector ${PROJECT_NAME}_generate_messages_cpp)
