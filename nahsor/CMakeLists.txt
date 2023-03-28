cmake_minimum_required(VERSION 3.8)
project(rm_nahsor)

add_compile_options(-Wno-dev)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set( CMAKE_BUILD_TYPE Release )

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(G2O REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

FIND_PACKAGE( CSparse  REQUIRED)
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )
include_directories(${CMAKE_CURRENT_BINARY_DIR})
set(dependencies
  rclcpp
  rclcpp_components
  cv_bridge
  OpenCV
  G2O
  Eigen3
  Ceres
)

# Include
include_directories(include)
include_directories(
        ${G2O_INCLUDE_DIRS}
        "/usr/include/eigen3"
)
# Create nahsor_modules lib
add_library(${PROJECT_NAME} SHARED
  "src/nahsor_node.cpp"
  "src/nahsor_prediction.cpp"
  "src/nahsor.cpp"
  "src/detector.cpp"
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} g2o_core g2o_stuff)

# #Q
# target_link_libraries(${PROJECT_NAME}${Protobuf_LIBRARIES})


# Create executable nodes
add_executable(nahsor "src/nahsor_main.cpp")
add_executable(vidtestpub "src/VidTestPub.cpp")
target_link_libraries(nahsor ${PROJECT_NAME})
target_link_libraries(vidtestpub ${PROJECT_NAME})




# export nahsor dependcy
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} eigen3_cmake_module)
rclcpp_components_register_nodes(${PROJECT_NAME} "nahsor::NahsorNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "nahsor::MinimalPublisher")
# Install launch res directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# # Install executable nodes
install(TARGETS nahsor
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS vidtestpub
DESTINATION lib/${PROJECT_NAME})

install(
   DIRECTORY launch
   DESTINATION share/${PROJECT_NAME}
)  

#test
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

ament_package()
