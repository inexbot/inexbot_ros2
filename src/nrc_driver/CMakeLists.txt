cmake_minimum_required(VERSION 3.8)
project(nrc_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nrc_msgs REQUIRED)


# SDK 路径
set(NRC_SDK_DIR /opt/nrc_sdk)

add_executable(nrc_node src/nrc_node.cpp)
ament_target_dependencies(nrc_node rclcpp std_msgs)
ament_target_dependencies(nrc_node rclcpp std_msgs sensor_msgs)
ament_target_dependencies(nrc_node rclcpp std_msgs sensor_msgs std_srvs)
ament_target_dependencies(nrc_node rclcpp std_msgs sensor_msgs std_srvs nrc_msgs)

target_link_libraries(nrc_node
  ${NRC_SDK_DIR}/lib/libnrc_host.so
  nlohmann_json::nlohmann_json)

target_include_directories(nrc_node PRIVATE
  ${NRC_SDK_DIR}/include)

target_link_libraries(nrc_node
  ${NRC_SDK_DIR}/lib/libnrc_host.so)

install(TARGETS
  nrc_node
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
