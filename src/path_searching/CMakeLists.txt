cmake_minimum_required(VERSION 3.0.2)
project(path_searching)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/../traj_opt/src DIR_TRAJ)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/../utils/src DIR_UTILS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/../map/src DIR_MAP)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/../control/src DIR_CONTROL)
aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_SOURCE)

message(STATUS ${PROJECT_SOURCE_DIR})
message(STATUS "HELLO")


find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  std_msgs
  utils
  traj_opt
  map
  control
)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

set(osqp_INCLUDE_DIR /usr/local/include/osqp)
set(OsqpEigen_INCLUDE_DIR /usr/local/include/OsqpEigen)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_searching
 CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../traj_opt/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../map/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../path_searching/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../control/include
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)


# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g -Wall")

add_library( path_searching 
    src/astar.cpp
    src/astar_replan.cpp
    ${DIR_TRAJ}
    ${DIR_UTILS}
    ${DIR_MAP}

    )
target_link_libraries( path_searching
    ${catkin_LIBRARIES} 
)  


add_executable(plan_node
    src/plan.cpp
    src/astar.cpp
    src/astar_replan.cpp
    ${DIR_TRAJ}
    ${DIR_UTILS}
    ${DIR_CONTROL}
)
target_link_libraries(plan_node
    ${catkin_LIBRARIES})