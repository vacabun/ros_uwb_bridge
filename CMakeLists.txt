cmake_minimum_required(VERSION 3.8)
project(ros_uwb_bridge)
# set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
# set(ENV{VCPKG_BINARY_SOURCES} "clear;default,readwrite")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(uwb_interfaces REQUIRED)

find_package(jsoncpp CONFIG REQUIRED)

add_executable(node
  src/node.cpp src/serial.cpp
)

target_include_directories(node PUBLIC include)

target_link_libraries(node JsonCpp::JsonCpp)

ament_target_dependencies(node rclcpp uwb_interfaces)

install(TARGETS node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
