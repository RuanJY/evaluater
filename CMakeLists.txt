cmake_minimum_required(VERSION 2.8.3)
project(evaluater)

add_compile_options(-std=c++14)
set(catkin_DIR /opt/ros/kinetic/share/catkin/cmake)
set(CMAKE_BUILD_TYPE "Release")

#ros
find_package(catkin REQUIRED COMPONENTS
        pcl_ros
        roscpp
        rospy
        sensor_msgs
        std_msgs
        )
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
#pcl
find_package(PCL REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(evaluater
        src/evaluater.cpp
        src/utility/tool.cpp
        )
target_link_libraries(evaluater ${catkin_LIBRARIES} ${PCL_LIBRARIES} )








