cmake_minimum_required(VERSION 2.8.3)

project(orb_slam2)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  message_filters
  cv_bridge
  cmake_modules
  dynamic_reconfigure 
  tf
  visual_slam_msgs
)

# Check C++11 or C++0x support
IF( NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native -fopenmp")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native -fopenmp")

  # Check C++11 or C++0x support
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX11)
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
     add_definitions(-DCOMPILEDWITHC11)
     message(STATUS "Using flag -std=c++11.")
  elseif(COMPILER_SUPPORTS_CXX0X)
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
     add_definitions(-DCOMPILEDWITHC0X)
     message(STATUS "Using flag -std=c++0x.")
  else()
     message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
ELSE()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCOMPILEDWITHC11)
  message(STATUS "Using flag -std=c++11.")
ENDIF()


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
LIST(APPEND CMAKE_MODULE_PATH /home/biorob/Pangolin/build/src/CMakeFiles/)

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin)
find_package(Boost REQUIRED COMPONENTS signals thread)

catkin_package()

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include/
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
    src/System.cc
    src/Tracking.cc
    src/LocalMapping.cc
    src/LoopClosing.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/Converter.cc
    src/MapPoint.cc
    src/KeyFrame.cc
    src/Map.cc
    src/Optimizer.cc
    src/PnPsolver.cc
    src/Frame.cc
    src/KeyFrameDatabase.cc
    src/Sim3Solver.cc
    src/Initializer.cc
    src/FramePublisher.cc
    src/MapPublisher.cc
)


IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET(DBoW2_LIB ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.dylib)
  SET(G2O_LIB ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.dylib)
ELSE()
  SET(DBoW2_LIB ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so)
  SET(G2O_LIB ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so)
ENDIF()

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${Pangolin_LIBRARIES}
	${EIGEN3_LIBS}
	${DBoW2_LIB}
  ${G2O_LIB}
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
#install(TARGETS ${PROJECT_NAME}
#        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


# Node for monocular camera
add_executable(ros_mono
	src/ros_mono.cc
)
target_link_libraries(ros_mono ${PROJECT_NAME})
