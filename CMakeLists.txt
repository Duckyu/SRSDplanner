cmake_minimum_required(VERSION 3.0.2)
project(srsd_planner)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(cmake_modules REQUIRED)  
find_package(Eigen3 REQUIRED)
# find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.54 REQUIRED)
find_package(octomap REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    mavros_msgs
    roslaunch
    cv_bridge
    pcl_conversions
    pcl_ros
    laser_geometry
    sensor_msgs
    message_generation
    message_filters
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  # CATKIN_DEPENDS roscpp rospy
  # DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

set(main_source
    src/exploration.hpp
    src/exploration.cpp
    src/octomap_utility.cpp
)
add_executable(main src/main.cpp ${main_source})
# add_dependencies(main srsd_planner_generate_messages_cpp)
target_link_libraries(main  ${EIGEN3_LIBS} ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES})

set(octomap_only_source
    src/octomap_only.hpp
    src/octomap_only.cpp
)
add_executable(octomap_only src/octomap_main.cpp ${octomap_only_source})
target_link_libraries(octomap_only  ${EIGEN3_LIBS} ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES})