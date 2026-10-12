cmake_minimum_required(VERSION 3.8)
project(network_bridge)

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
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZSTD REQUIRED libzstd)
pkg_check_modules(ZMQ REQUIRED IMPORTED_TARGET libzmq)
find_path(CPPZMQ_INCLUDE_DIR zmq.hpp HINTS ${ZMQ_INCLUDE_DIRS})
if(NOT CPPZMQ_INCLUDE_DIR)
  message(FATAL_ERROR "cppzmq headers not found; install the libzmq3-dev rosdep dependency")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_core test/test_core.cpp TIMEOUT 10)
  if(TARGET test_core)
    target_include_directories(test_core PRIVATE include)
    target_link_libraries(test_core Threads::Threads)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(NETWORK_BRIDGE_LAUNCH_TEST_TIMEOUT 30 CACHE STRING "Timeout in seconds for launch tests")

  add_launch_test(
    test/test_udp.py
    TIMEOUT ${NETWORK_BRIDGE_LAUNCH_TEST_TIMEOUT}
  )
  add_launch_test(
    test/test_tcp.py
    TIMEOUT ${NETWORK_BRIDGE_LAUNCH_TEST_TIMEOUT}
  )
  add_launch_test(
    test/test_zmq.py
    TIMEOUT 60
  )
endif()

include_directories(include)

add_executable(network_bridge
    src/network_bridge.cpp
    src/subscription_manager.cpp
    src/subscription_manager_tf.cpp
)

add_library(udp_interface SHARED
  src/network_interfaces/udp_interface.cpp
)

add_library(tcp_interface SHARED
  src/network_interfaces/tcp_interface.cpp
)

add_library(zmq_interface SHARED
  src/network_interfaces/zmq_interface.cpp
)

target_link_libraries(network_bridge PUBLIC
  ${std_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${ZSTD_LIBRARIES}
)

target_link_libraries(udp_interface PUBLIC
  pluginlib::pluginlib
  rclcpp::rclcpp
  Boost::headers
  Threads::Threads
)

target_link_libraries(tcp_interface PUBLIC
  pluginlib::pluginlib
  rclcpp::rclcpp
  Boost::headers
  Threads::Threads
)

target_link_libraries(zmq_interface PUBLIC
  PkgConfig::ZMQ
  rclcpp::rclcpp
  pluginlib::pluginlib
  Threads::Threads
)

target_include_directories(zmq_interface PUBLIC ${CPPZMQ_INCLUDE_DIR})

if(BUILD_TESTING)
  ament_add_gtest(test_zmq_interface test/test_zmq_interface.cpp TIMEOUT 15)
  if(TARGET test_zmq_interface)
    target_link_libraries(test_zmq_interface zmq_interface Boost::headers)
  endif()
endif()

pluginlib_export_plugin_description_file(network_bridge network_interface_plugins.xml)

install(TARGETS
  network_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  udp_interface
  tcp_interface
  zmq_interface
  DESTINATION lib/
)

install(DIRECTORY
  config
  launch
  demo
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
