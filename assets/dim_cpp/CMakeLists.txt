# message(WARNING "Processing CMakeLists.txt for dim_cpp")

cmake_minimum_required(VERSION 3.5)
project(dim_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# message(WARNING "---------------------------------------------------------------------")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(xarm_planner REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  ${xarm_planner_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# message(WARNING "xarm_planner include dirs: ${xarm_planner_INCLUDE_DIRS}")
# message(WARNING "xarm_planner libraries: ${xarm_planner_LIBRARIES}")
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(moveit_converter src/moveit_converter.cpp)
add_executable(point_cloud_transformer src/point_cloud_transformer.cpp)
add_executable(transform_publisher src/transform_publisher.cpp)
add_executable(data_recorder src/data_recorder.cpp)
target_include_directories(moveit_converter PUBLIC
  ${xarm_planner_INCLUDE_DIRS}
)

# Link against the required libraries
target_link_libraries(moveit_converter
  ${xarm_planner_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
  ${moveit_visual_tools_LIBRARIES}
  ${pluginlib_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${moveit_msgs_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(point_cloud_transformer
  rclcpp
  tf2_ros
  tf2_sensor_msgs
  sensor_msgs
  Eigen3
)

ament_target_dependencies(moveit_converter
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  pluginlib
  geometry_msgs
  moveit_msgs
  Eigen3
  graph_msgs
  pluginlib
  xarm_msgs
  xarm_planner
  moveit
)

ament_target_dependencies(transform_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(data_recorder
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  PCL
  pcl_conversions
  pcl_ros
)

install(TARGETS
  moveit_converter
  point_cloud_transformer
  transform_publisher
  data_recorder
  DESTINATION lib/${PROJECT_NAME})

# Install the launch files, if any
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY piper_description
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY srdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
