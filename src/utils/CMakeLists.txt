cmake_minimum_required(VERSION 3.0.2)
project(utils)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/../traj_opt/src DIR_TRAJ)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/../utils/src DIR_UTILS)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  traj_opt
)

find_package(Eigen3 REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES utils
 CATKIN_DEPENDS traj_opt
)


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../traj_opt/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils/include
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")


add_library(utils 
    ${DIR_TRAJ}
)
target_link_libraries( utils 
    ${catkin_LIBRARIES} 
)  

