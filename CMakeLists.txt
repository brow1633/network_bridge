cmake_minimum_required(VERSION 3.8)
project(ros2_tether)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost 1.65 REQUIRED COMPONENTS system iostreams)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
pkg_check_modules(ZSTD REQUIRED libzstd)

include_directories(${Boost_INCLUDE_DIRS} ${ZSTD_INCLUDE_DIRS})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_launch_test(
    test/test_udp.py
    TIMEOUT 2  # Sets a timeout for the test in seconds
  )
  add_launch_test(
    test/test_tcp.py
    TIMEOUT 2  # Sets a timeout for the test in seconds
  )
endif()

include_directories(include)

add_executable(ros2_tether src/ros2_tether.cpp src/subscription_manager.cpp)

add_library(udp_interface SHARED
  src/network_interfaces/udp_interface.cpp
)

add_library(tcp_interface SHARED
  src/network_interfaces/tcp_interface.cpp
)

ament_target_dependencies(ros2_tether pluginlib rclcpp std_msgs)
ament_target_dependencies(udp_interface pluginlib rclcpp)
ament_target_dependencies(tcp_interface pluginlib rclcpp)

target_link_libraries(ros2_tether ${ZSTD_LIBRARIES})
target_link_libraries(udp_interface ${Boost_LIBRARIES})
target_link_libraries(tcp_interface ${Boost_LIBRARIES})

pluginlib_export_plugin_description_file(ros2_tether network_interface_plugins.xml)

install(TARGETS
  ros2_tether
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  udp_interface
  tcp_interface
  DESTINATION lib/
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
