cmake_minimum_required(VERSION 3.8)
project(ugv_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(PkgConfig REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

pkg_check_modules(GPIOD REQUIRED libgpiodcxx)

# install python nodes
install(PROGRAMS 
        ugv_description/xbox_teleop.py 
        ugv_description/ap_odom_bridge.py
        ugv_description/imu_data_transformer.py
 DESTINATION lib/${PROJECT_NAME})

add_executable(wheel_odom src/wheel_odom.cpp) # Naming in progress.
target_include_directories(wheel_odom PRIVATE ${GPIOD_INCLUDE_DIRS})
target_link_libraries(wheel_odom ${GPIOD_LIBRARIES})

ament_target_dependencies(wheel_odom rclcpp std_msgs geometry_msgs)

install(TARGETS
    wheel_odom
    DESTINATION lib/${PROJECT_NAME})

# install the dependencies to the directories
install(
  DIRECTORY
    launch
    urdf
    meshes
    config
    worlds
  DESTINATION share/${PROJECT_NAME}
)

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
