cmake_minimum_required(VERSION 2.8.3)
project(baxter_pickup)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  block_grasp_generator
  baxter_core_msgs
  std_msgs
  geometry_msgs
  baxter_control
  image_transport
  cv_bridge
  roscpp
  tf
  baxter_gripper_server
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system thread)
catkin_package(
  INCLUDE_DIRS #include
)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")
set(CMAKE_BUILD_TYPE Debug)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(viz_servo src/viz_servo.cpp)
target_link_libraries(viz_servo ${catkin_LIBRARIES} ${Boost_LIBRARIES})


