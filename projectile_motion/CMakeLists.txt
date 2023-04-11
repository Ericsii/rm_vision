cmake_minimum_required(VERSION 3.8)
project(projectile_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------- Find thirdparty library -------- #
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rmoss_projectile_motion REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(rm_interfaces REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  message_filters
  tf2_ros
  tf2_geometry_msgs
  rmoss_projectile_motion
  auto_aim_interfaces
  rm_interfaces
)

# ------------- Build library ------------- #
add_library(${PROJECT_NAME} SHARED
  src/projectile_motion_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "projectile_motion::ProjectileMotionNode"
  EXECUTABLE projectile_motion_node
)

# ---------------- Install --------------- #
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install  directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ----------------- Test ----------------- #
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
