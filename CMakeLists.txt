cmake_minimum_required(VERSION 2.8.3)
project(exploration_rover_i)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  message_generation
)

add_message_files(
  FILES
  motor_ror.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(
  motor_driver_test
    src/motor_driver_test.cpp
    src/motor_driver.cpp
)
target_link_libraries(motor_driver_test
  ${catkin_LIBRARIES}
)

add_executable(
  motor_driver_ros
    src/motor_driver_ros.cpp
    src/motor_driver.cpp
)
target_link_libraries(motor_driver_ros
  ${catkin_LIBRARIES}
)

add_executable(
  controller_simple
    src/controller_simple.cpp
)
target_link_libraries(controller_simple
  ${catkin_LIBRARIES}
)