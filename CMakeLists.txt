cmake_minimum_required(VERSION 3.10)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

project(xarm6_octomap_avoidance)

# このパッケージ内のテストは常に無効
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(CATKIN_ENABLE_TESTING OFF CACHE BOOL "" FORCE)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  moveit_commander
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})