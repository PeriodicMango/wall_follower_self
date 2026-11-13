cmake_minimum_required(VERSION 3.16)
project(wall_follower_2_3 LANGUAGES C CXX)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(wall_follower_2_3 src/wall_follower_2_3.cpp)
target_compile_features(wall_follower_2_3 PRIVATE cxx_std_17)
target_include_directories(wall_follower_2_3 PRIVATE include)
ament_target_dependencies(wall_follower_2_3 rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2)
if(APPLE)
  # The course distribution's ROS message libraries load Python generator symbols.
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
  target_link_options(wall_follower_2_3 PRIVATE "LINKER:-needed_library,$<TARGET_FILE:Python3::Python>")
  set_target_properties(wall_follower_2_3 PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$<TARGET_FILE_DIR:Python3::Python>")
endif()
