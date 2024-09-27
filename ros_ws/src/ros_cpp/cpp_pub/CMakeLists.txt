cmake_minimum_required(VERSION 3.8)
project(cpp_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_cpp_proxy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(str_publisher SHARED
  src/str_publisher.cpp
)
ament_target_dependencies(str_publisher
  ros_cpp_proxy
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_node(str_publisher
  PLUGIN "publisher::StrPublisher"
  EXECUTABLE str_publisher_node
)
target_include_directories(str_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_export_targets(str_publishertargets HAS_LIBRARY_TARGET)
install(TARGETS str_publisher EXPORT str_publishertargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
