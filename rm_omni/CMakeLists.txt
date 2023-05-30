cmake_minimum_required(VERSION 3.11)
project(rm_omni)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------- Find thirdparty library -------- #
# ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_transport_plugins REQUIRED)
find_package(tf2 REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(rmoss_util REQUIRED)
find_package(openvino_armor_detector REQUIRED)

set(ros_dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  visualization_msgs
  cv_bridge
  image_transport
  image_transport_plugins
  tf2
  auto_aim_interfaces
  rmoss_util
  rmoss_interfaces
  openvino_armor_detector
)

# third party packages
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED core imgproc video dnn)
find_package(OpenVINO REQUIRED COMPONENTS Runtime ONNX)
find_package(fmt REQUIRED)

set(other_dependencies
  Eigen3::Eigen
  ${OpenCV_LIBS}
  openvino::frontend::onnx
  openvino::runtime
  fmt::fmt
)

# ------------- Build library ------------- #
add_library(${PROJECT_NAME} SHARED
  src/omni_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  ${other_dependencies}
)
ament_target_dependencies(${PROJECT_NAME} ${ros_dependencies})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "rm_omni::OmniNode"
  EXECUTABLE omni_node
)

# ---------------- Install --------------- #
install(DIRECTORY include/
  DESTINATION include
)

# Install  directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ----------------- Test ----------------- #
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${ros_dependencies} ${other_dependencies})

ament_package()
