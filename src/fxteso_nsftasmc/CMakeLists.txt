cmake_minimum_required(VERSION 3.0.2)
project(fxteso_nsftasmc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
)

catkin_package(
CATKIN_DEPENDS gazebo_msgs geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf2
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_3_INCLUDE_DIRS}
)

add_executable(position_ctrl_node src/pos_ctrl.cpp)
target_link_libraries(position_ctrl_node ${catkin_LIBRARIES})

add_executable(attitude_ctrl_node src/att_ctrl.cpp)
target_link_libraries(attitude_ctrl_node ${catkin_LIBRARIES})

add_executable(pose_references_node src/poseRef.cpp)
target_link_libraries(pose_references_node ${catkin_LIBRARIES})

add_executable(quad_dyn_node src/quad_dynamics.cpp)
target_link_libraries(quad_dyn_node ${catkin_LIBRARIES})

add_executable(quad_gazebo_comm_node src/quad_gazebo_com.cpp)
target_link_libraries(quad_gazebo_comm_node ${catkin_LIBRARIES})

add_executable(fxteso_zhang_node src/fxteso_zhang.cpp)
target_link_libraries(fxteso_zhang_node ${catkin_LIBRARIES})
