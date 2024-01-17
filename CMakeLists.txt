cmake_minimum_required(VERSION 3.8)
project(uwb_ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uwb_interfaces REQUIRED)

add_executable(bridge
    src/bridge.cpp
)

target_include_directories(bridge PUBLIC include/)
target_link_libraries(bridge JsonCpp::JsonCpp)
ament_target_dependencies(bridge rclcpp uwb_interfaces)

install(TARGETS bridge DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
