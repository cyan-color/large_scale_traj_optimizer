cmake_minimum_required(VERSION 2.8.3)

project(example1)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ../include
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_receiver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_executable(${PROJECT_NAME}_node src/example1_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_rviz_node src/example_rviz.cpp)

target_link_libraries(${PROJECT_NAME}_rviz_node
  ${catkin_LIBRARIES}
)
