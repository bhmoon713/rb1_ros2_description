cmake_minimum_required(VERSION 3.8)
project(rb1_ros2_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(wait_for_service_node src/wait_for_service.cpp)
ament_target_dependencies(wait_for_service_node rclcpp controller_manager_msgs)
install(TARGETS
  wait_for_service_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    launch
    meshes
	urdf
	xacro
	config
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()