cmake_minimum_required(VERSION 3.15.1)
project(ros_dynamic_topic_synchronizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options("-Wno-deprecated")
  add_compile_options("-Wsign-compare")
  add_compile_options("-Werror=return-type")
  add_compile_options("-Werror=return-local-addr")
  # error in the constructor when initializing a member with itself in the initalizer list.
  add_compile_options("-Werror=init-self")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
    include
)

ament_auto_add_library(ros_dynamic_topic_synchronizer
  src/topic_synchronizer.cpp
)

ament_auto_add_executable(example_listener_node
  example/example_listener_node_ros2.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
)