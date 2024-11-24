cmake_minimum_required(VERSION 3.8)
project(detection_obstacle_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros)
find_package(tf2)

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

add_executable(minimal_opencv_ros2_node src/minimal_opencv_ros2_node.cpp)
ament_target_dependencies(minimal_opencv_ros2_node rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)


add_executable(yaw_publisher_node src/yaw_publisher.cpp)
ament_target_dependencies(yaw_publisher_node rclcpp std_msgs sensor_msgs tf2 tf2_ros) 

install(TARGETS yaw_publisher_node
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS
   minimal_opencv_ros2_node
   DESTINATION lib/${PROJECT_NAME}
)

# Python scripts installation
install(PROGRAMS scripts/detection_obstacle_node.py
DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/opencv_decoder.py
DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(
  detection_obstacle_pkg
  PACKAGE_DIR src/detection_obstacle_pkg
)

ament_package()
