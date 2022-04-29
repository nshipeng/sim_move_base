cmake_minimum_required(VERSION 3.0.2)
project(control)


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
find_package(OsqpEigen REQUIRED)

set(osqp_INCLUDE_DIR /usr/local/include/osqp)
set(OsqpEigen_INCLUDE_DIR /usr/local/include/OsqpEigen)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES control
 CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../traj_opt/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../map/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../control/include
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${OsqpEigen_INCLUDE_DIR}
    ${osqp_INCLUDE_DIR}
)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library( control 
    src/rk4.cpp
    # src/mpc_control.cpp
    ${DIR_TRAJ}
    ${DIR_UTILS}

    )
target_link_libraries( control
    ${catkin_LIBRARIES} 
    osqp::osqp
    OsqpEigen::OsqpEigen
    )  


# add_executable(rk_node
#     src/rk_node.cpp
#     src/rk4.cpp
#     ${DIR_TRAJ}
#     ${DIR_UTILS}
#     )
# target_link_libraries(rk_node
#     ${catkin_LIBRARIES})


# add_executable(mpc_test_node
#     src/mpc_test.cpp
#      src/mpc_control.cpp
#     ${DIR_TRAJ}
#     ${DIR_UTILS}
#     )
# target_link_libraries(mpc_test_node
#     ${catkin_LIBRARIES}
#     osqp::osqp
#     OsqpEigen::OsqpEigen
#     )



