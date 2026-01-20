cmake_minimum_required(VERSION 3.8)
project(ros2_lectures_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ShortCmd.msg"
  DEPENDENCIES std_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
ament_export_dependencies(rosidl_default_runtime)


add_executable(vel_pub_example src/vel_pub_example.cpp)

ament_target_dependencies(vel_pub_example rclcpp geometry_msgs)

install(TARGETS vel_pub_example DESTINATION lib/${PROJECT_NAME})


add_executable(pos_sub_example src/pos_sub_example.cpp)

ament_target_dependencies(pos_sub_example rclcpp turtlesim)

install(TARGETS pos_sub_example DESTINATION lib/${PROJECT_NAME})



add_executable(shortcmd_pub_example src/shortcmd_pub_example.cpp)

ament_target_dependencies(shortcmd_pub_example rclcpp geometry_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(shortcmd_pub_example  "${cpp_typesupport_target}")

install(TARGETS shortcmd_pub_example DESTINATION lib/${PROJECT_NAME})

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

ament_package()
