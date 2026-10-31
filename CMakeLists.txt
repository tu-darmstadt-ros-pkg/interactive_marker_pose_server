cmake_minimum_required(VERSION 3.16)
project(interactive_marker_pose_server)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ------------------------------------------------------------------------------
# Library with the actual implementation
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME} src/interactive_marker_pose_server.cpp)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC rclcpp::rclcpp
         ${std_msgs_TARGETS}
         ${geometry_msgs_TARGETS}
         ${visualization_msgs_TARGETS}
         interactive_markers::interactive_markers
         tf2::tf2
         tf2_ros::tf2_ros
         tf2_eigen::tf2_eigen
         tf2_geometry_msgs::tf2_geometry_msgs
         Eigen3::Eigen)

# ------------------------------------------------------------------------------
# Node executable (just main())
# ------------------------------------------------------------------------------
add_executable(interactive_marker_pose_server_node
               src/interactive_marker_pose_server_node.cpp)

target_link_libraries(interactive_marker_pose_server_node ${PROJECT_NAME})

# ------------------------------------------------------------------------------
# Install targets and resources
# ------------------------------------------------------------------------------
install(
  TARGETS ${PROJECT_NAME} interactive_marker_pose_server_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(hector_testing_utils REQUIRED)

  ament_add_gtest(test_interactive_marker_pose_server
                  test/test_interactive_marker_pose_server.cpp TIMEOUT 120)
  target_link_libraries(test_interactive_marker_pose_server ${PROJECT_NAME}
                        hector_testing_utils::hector_testing_utils)
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  interactive_markers
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  Eigen3)

ament_package()
