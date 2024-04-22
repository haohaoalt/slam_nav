cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()
add_definitions(-w)
MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3  ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 ")

# Check C++14 or C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC14)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

# set(cv_bridge_DIR /home/xin/project_ws2/devel/share/cv_bridge/cmake)
# 要保证整个工程的opencv版本一致，包括dbow，源码以及ros相关的
# 3 4 都可以正常运行
find_package(OpenCV)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 3(4) not found.")
   endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        message_generation
        tf
        )
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${PROJECT_SOURCE_DIR}/../../../include/CameraModels
${PROJECT_SOURCE_DIR}/../../../Thirdparty/Sophus
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PCL_LIBRARIES}
${catkin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
#编辑正常SLAM模式的打开这个
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
-lboost_system
)

## Node for stereo camera
#rosbuild_add_executable(Stereo
#src/ros_stereo.cc
#)
#
#target_link_libraries(Stereo
#${LIBS}
#)

# Node for RGB-D camera
rosbuild_add_executable(Akm_RGBD
src/ros_akm_rgbd.cc
)

target_link_libraries(Akm_RGBD
${LIBS}
)


## Node for RGB-D gazebo
#rosbuild_add_executable(kinect
#src/kinect_rgbd.cc
#)
#
#target_link_libraries(kinect
#${LIBS}
#)
#
## Node for stereo-inertial camera
#rosbuild_add_executable(Stereo_Inertial
#src/ros_stereo_inertial.cc
#)
#
#target_link_libraries(Stereo_Inertial
#${LIBS}
#)
