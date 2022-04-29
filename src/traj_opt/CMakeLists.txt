cmake_minimum_required(VERSION 3.0.2)
project(traj_opt)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(Eigen3 REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_opt
 CATKIN_DEPENDS
)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/../traj_opt/src  DIR_TRAJ)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../traj_opt/include
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library(traj_opt
    ${DIR_TRAJ}
)

target_link_libraries(traj_opt
    ${catkin_LIBRARIES} 
    )  
