################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(wall_follower)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

################################################################################
# Build
################################################################################

include_directories(include)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "rclpy"
  "sensor_msgs"
  "tf2"
)

# 每个版本各自编成一个可执行文件；现场用 `ros2 run wall_follower <名字>` 切换，不用重新 build。
# 原来的 add_executable(${EXEC_NAME} src/wall_follower.cpp) 已去掉：
# 仓库里 src/wall_follower.cpp 已改名为 src/wall_follower_2.cpp。
set(ALT_VERSIONS
  wall_follower_1
  wall_follower_1_1
  wall_follower_2
  wall_follower_2_1
  wall_follower_2_2
)

foreach(v IN LISTS ALT_VERSIONS)
  add_executable(${v} src/${v}.cpp)
  ament_target_dependencies(${v} ${dependencies})
endforeach()

################################################################################
# Install
################################################################################
install(TARGETS ${ALT_VERSIONS}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/see_marker.py
  scripts/point_transformer.py
  DESTINATION lib/${PROJECT_NAME}
)


################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)

ament_package()
