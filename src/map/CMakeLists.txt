cmake_minimum_required(VERSION 3.0.2)
project(map)


aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/../traj_opt/src DIR_TRAJ)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/../utils/src DIR_UTILS)
aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_SOURCE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  utils
  traj_opt
)

find_package(Eigen3 REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES map
 CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../traj_opt/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../map/include
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library( map 
    src/int8_map.cpp
    ${DIR_TRAJ}
    ${DIR_UTILS}

    )
target_link_libraries( map
    ${catkin_LIBRARIES} 
    )  

add_executable(map_test_node
    src/test.cpp 
    ${DIR_SOURCE}
    ${DIR_TRAJ}
    ${DIR_UTILS}
    )
target_link_libraries(map_test_node
    ${catkin_LIBRARIES})





