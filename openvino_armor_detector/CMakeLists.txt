cmake_minimum_required(VERSION 3.8)
project(openvino_armor_detector)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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
  src/openvino_detector.cpp
  src/mono_measure_tool.cpp
  src/openvino_detect_node.cpp
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
  PLUGIN "rm_auto_aim::OpenVINODetectNode"
  EXECUTABLE detector_node
)

# ---------------- Install --------------- #
install(DIRECTORY include/
  DESTINATION include
)

# Install  directories
install(DIRECTORY launch config model
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
  add_subdirectory(test)
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${ros_dependencies})

ament_package()
