cmake_minimum_required(VERSION 3.8)
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
