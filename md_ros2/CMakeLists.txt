cmake_minimum_required(VERSION 3.5)
project(md_ros2)

# Compile as C++11, supported in ROS2
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(tf2 REQUIRED)
find_package(md_msgs REQUIRED)

# ament_package()
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# Add the source files to the build
set(SOURCES
  src/com.cpp
  src/robot.cpp
  src/motor_init.cpp
)

# Build
add_executable(${PROJECT_NAME} src/main.cpp ${SOURCES})
ament_target_dependencies(${PROJECT_NAME}
 rclcpp
 sensor_msgs
 std_msgs
 geometry_msgs
 md_msgs
)

add_executable(joy_test src/joy_test.cpp)
ament_target_dependencies(joy_test rclcpp std_msgs sensor_msgs geometry_msgs)
target_link_libraries(joy_test ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

add_executable(webot_test src/webot_test.cpp)
ament_target_dependencies(webot_test rclcpp std_msgs sensor_msgs geometry_msgs)
target_link_libraries(webot_test ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})



# Install
install(TARGETS
  ${PROJECT_NAME}
  joy_test
  webot_test

  DESTINATION lib/${PROJECT_NAME})

# Install other files needed by the package
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# Test
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