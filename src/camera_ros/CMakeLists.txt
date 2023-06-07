cmake_minimum_required(VERSION 3.5)
project(camera_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_FLAGS_RELEASE "-Wno-deprecated-declarations -O2")

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(
    include
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}
    message(${PROJECT_SOURCE_DIR})
    /usr/lib/
    /usr/include/
)

link_directories(
	/usr/lib/aarch64-linux-gnu
  /usr/aarch64-linux-gnu/lib/
  /usr/include/
  /usr/local/lib
  ${PROJECT_SOURCE_DIR}
)

set (executable_name camera_ros_node)
add_executable(${executable_name} main.cpp)
ament_target_dependencies(${executable_name} rclcpp std_msgs sensor_msgs OpenCV cv_bridge)

install(DIRECTORY launch params rviz_cfg
		DESTINATION share/${PROJECT_NAME})

install(TARGETS
	${executable_name}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
