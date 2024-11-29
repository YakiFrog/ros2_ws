cmake_minimum_required(VERSION 3.8)
project(bt_sample)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(
  server src/server.cpp
)

ament_auto_add_executable(
  bt_node src/bt_node.cpp include/action_node.hpp include/ros_node.hpp
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CallText.srv"
  DEPENDENCIES
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(server "${cpp_typesupport_target}")
target_link_libraries(bt_node "${cpp_typesupport_target}")

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
