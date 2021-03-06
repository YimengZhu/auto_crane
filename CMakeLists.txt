
cmake_minimum_required(VERSION 3.1.3)
project(auto_crane)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS
  doc/interactivity/include
)


add_message_files(
  FILES
  waypoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    message_runtime
  DEPENDS
    EIGEN3
)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(planner src/moveit_plan.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planner DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})