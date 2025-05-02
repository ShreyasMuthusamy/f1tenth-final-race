cmake_minimum_required(VERSION 3.8)
project(opponent_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
foreach (PACKAGE
        ament_cmake
        rclcpp
        geometry_msgs
        nav_msgs
        ackermann_msgs
        sensor_msgs
        visualization_msgs
        std_msgs)

    find_package(${PACKAGE} REQUIRED)

endforeach ()

include_directories(include)

add_executable(opponent_estimator_node src/opponent_estimator_node.cpp src/opponent_estimator.cpp src/obs_detection_utils.cpp src/visualization.cpp)
ament_target_dependencies(opponent_estimator_node
        rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs visualization_msgs std_msgs)
target_include_directories(opponent_estimator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(opponent_estimator_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS opponent_estimator_node
  DESTINATION lib/${PROJECT_NAME})

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
