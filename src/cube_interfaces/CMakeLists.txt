cmake_minimum_required(VERSION 3.8)
project(cube_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cube_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  # ${Boost INCLUDE_DIRS}
)

file(GLOB SRC src/*.cpp src/*/*.cpp)

add_library(CUBE_INTERFACES SHARED ${SRC})

target_include_directories(CUBE_INTERFACES PUBLIC
  include
  /usr/local/include
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
)

ament_target_dependencies(CUBE_INTERFACES
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cube_msgs
)

add_executable(cube_interfaces_node src/cube_interfaces_node.cpp src/controller.cpp)

target_link_libraries(cube_interfaces_node CUBE_INTERFACES)

install(TARGETS
  cube_interfaces_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
