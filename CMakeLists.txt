cmake_minimum_required(VERSION 3.8)
project(robot_arm_bringup)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# ── Custom messages ─────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoCmd.msg"
  "msg/RgbLed.msg"
  DEPENDENCIES std_msgs
)

# ── Python nodes ─────────────────────────────────────────────────────────────
install(
  PROGRAMS
    robot_arm_bringup/servo_node.py
    robot_arm_bringup/rgb_led_node.py
    robot_arm_bringup/bt_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
