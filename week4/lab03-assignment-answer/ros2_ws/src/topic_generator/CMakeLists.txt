cmake_minimum_required(VERSION 3.8)
project(topic_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(data_receiver src/data_receiver.cpp)
target_include_directories(data_receiver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(data_receiver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  data_receiver
  "rclcpp"
  "std_msgs"
)

add_executable(info_sender src/info_sender.cpp)
target_include_directories(info_sender PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(info_sender PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  info_sender
  "rclcpp"
  "std_msgs"
)

install(TARGETS data_receiver info_sender
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
