# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(uvc)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Find wiringPi
find_library(wiringPi_LIB wiringPi)

## Find CPPRESTSDK
find_package(cpprestsdk REQUIRED)

## Declare ROS messages and services
add_service_files(FILES set_lamp.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${WIRINGPI_INCLUDE_DIRS})

# %Tag(SRVCLIENT)%

add_executable(lamp_control src/lamp_node.cpp)
target_link_libraries(lamp_control ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${wiringPi_LIB})
add_dependencies(lamp_control uvc_gencpp)

add_executable(rest src/rest_node.cpp)
target_link_libraries(rest ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} cpprestsdk::cpprest)
add_dependencies(rest uvc_gencpp)

# %EndTag(SRVCLIENT)%

# %EndTag(FULLTEXT)%
