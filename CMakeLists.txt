cmake_minimum_required(VERSION 3.8)
project(odrive_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlMessage.msg"
  "msg/ControllerStatus.msg"
  "msg/ODriveStatus.msg"
  "srv/AxisState.srv"
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_executable(odrive_can_node 
  src/epoll_event_loop.cpp
  src/socket_can.cpp
  src/odrive_can_node.cpp
  src/main.cpp
  include/odrive_can_node.hpp)

ament_target_dependencies(odrive_can_node
  rclcpp
)

target_compile_features(odrive_can_node PRIVATE cxx_std_20)

add_library(odrive_hardware_interface SHARED
  src/epoll_event_loop.cpp
  src/odrive_hardware_interface.cpp
  src/socket_can.cpp
)
target_include_directories(odrive_hardware_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(odrive_hardware_interface ${THIS_PACKAGE_INCLUDE_DEPENDS} hardware_interface pluginlib rclcpp rclcpp_lifecycle)
rosidl_target_interfaces(odrive_hardware_interface
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_targets(odrive_hardware_interfaceTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_interface.xml)

install(
  TARGETS odrive_can_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS odrive_hardware_interface
  EXPORT odrive_hardware_interfaceTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(odrive_can_node "${cpp_typesupport_target}")

ament_package()
