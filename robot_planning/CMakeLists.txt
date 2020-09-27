cmake_minimum_required(VERSION 2.8.3)
project(robot_planning)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  move_base_msgs
  tf
  map_msgs
  message_generation 
  nav_msgs 
  roslib 
)

add_message_files(
  FILES
  trajData.msg
  state.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

add_compile_options(-std=c++11)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES $(PROJECT_NAME)
	CATKIN_DEPENDS nav_msgs roscpp message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_control src/robot_control.cpp)
add_executable(test_control src/test_control.cpp)
add_executable(pub_map_odom_tf src/main_pub_map_odom.cpp)
add_executable(planner src/planner.cpp)
add_executable(visualizer src/visualizer.cpp)
add_executable(visualization_tutorial src/visualization_tutorial.cpp)

target_link_libraries(robot_control ${catkin_LIBRARIES})
target_link_libraries(test_control ${catkin_LIBRARIES})
target_link_libraries(pub_map_odom_tf ${catkin_LIBRARIES})
target_link_libraries(planner ${catkin_LIBRARIES})
target_link_libraries(visualizer ${catkin_LIBRARIES})
target_link_libraries(visualization_tutorial ${catkin_LIBRARIES})

add_dependencies(planner ${catkin_EXPORTED_TARGETS})
add_dependencies(robot_control ${catkin_EXPORTED_TARGETS})
add_dependencies(test_control ${catkin_EXPORTED_TARGETS})
add_dependencies(visualizer ${catkin_EXPORTED_TARGETS})
add_dependencies(visualization_tutorial ${catkin_EXPORTED_TARGETS})