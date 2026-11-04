cmake_minimum_required(VERSION 3.8)
project(ai_robotic_joint_ros2)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ManualCommand.msg"
  "msg/ControlTelemetry.msg"
  "msg/StudyCommand.msg"
  "msg/StudyTelemetry.msg"
)

add_library(joint_controller_plugin SHARED
  src/JointControllerPlugin.cpp
  src/ManualJointControllerPlugin.cpp
  src/StudyJointControllerPlugin.cpp
)

target_compile_features(joint_controller_plugin PUBLIC cxx_std_17)
target_include_directories(joint_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(joint_controller_plugin
  pluginlib
  rclcpp
  sensor_msgs
  std_msgs
  webots_ros2_driver
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(joint_controller_plugin "${cpp_typesupport_target}")

pluginlib_export_plugin_description_file(
  webots_ros2_driver
  plugin_description.xml
)

install(
  TARGETS joint_controller_plugin
  EXPORT export_joint_controller_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch resource worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/ros2_joint_experiment.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/joint_safety_node.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/manual_panel_server.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/manual_panel_core.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/study_panel_server.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/study_panel_core.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY study_web DESTINATION share/${PROJECT_NAME})
install(DIRECTORY web DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  enable_testing()
  add_executable(study_pair_test tests/study_pair_test.cpp)
  target_compile_features(study_pair_test PUBLIC cxx_std_17)
  target_include_directories(study_pair_test PRIVATE include)
  add_test(NAME study_pair_test COMMAND study_pair_test)
endif()

ament_export_include_directories(include)
ament_export_libraries(joint_controller_plugin)
ament_export_targets(export_joint_controller_plugin HAS_LIBRARY_TARGET)

ament_package()
