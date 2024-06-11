cmake_minimum_required(VERSION 3.15.1)
project(fsd_topic_synchronizer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options("-Wno-deprecated")
    #add_compile_options("-Wsign-compare")
    add_compile_options("-Werror=return-type")
    add_compile_options("-Werror=return-local-addr")
    # error in the constructor when initializing a member with itself in the initalizer list.
    add_compile_options("-Werror=init-self")
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    rospy
    std_msgs
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES fsd_topic_synchronizer
    CATKIN_DEPENDS roscpp sensor_msgs 
    #DEPENDS 
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_library(fsd_topic_synchronizer
    src/topic_synchronizer.cpp
    )

target_link_libraries(fsd_topic_synchronizer
    ${catkin_LIBRARIES}
    )

add_executable(example_listener_node
    example/example_listener_node.cpp
    )

add_dependencies(example_listener_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(example_listener_node
    ${catkin_LIBRARIES}
    fsd_topic_synchronizer
    )

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
    )

# Add tests

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED rostest)

  catkin_add_gtest(approx_time_policy_test test/approx_time_policy_test.cpp)
  target_link_libraries(approx_time_policy_test fsd_common ${catkin_LIBRARIES})
endif() # CATKIN_ENABLE_TESTING
