cmake_minimum_required(VERSION 3.0.2)
project(serial_demo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmsg
  nav_msgs
  std_msgs
  serial
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_demo src/serial_demo.cpp)
target_link_libraries(serial_demo ${catkin_LIBRARIES})

