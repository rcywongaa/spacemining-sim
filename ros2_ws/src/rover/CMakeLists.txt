cmake_minimum_required(VERSION 3.8)
project(rover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rover_interfaces REQUIRED)
find_package(angles REQUIRED)

# https://stackoverflow.com/questions/48169653/finding-geographiclib-in-cmake-on-debian/48524106#48524106
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package (GeographicLib REQUIRED)
include_directories (${GeographicLib_INCLUDE_DIRS})

add_executable (${PROJECT_NAME} src/main.cpp)

# ament_target_dependencies(${PROJECT_NAME} rclcpp rover_interfaces ${GeographicLib_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} rclcpp rover_interfaces angles rclcpp_action)
target_link_libraries(${PROJECT_NAME} ${GeographicLib_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
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
