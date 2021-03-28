cmake_minimum_required(VERSION 3.0.2)
project(dtb_quadruped_core)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   CoreService.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(service_core src/service_core.cpp)
target_link_libraries(service_core ${catkin_LIBRARIES})
add_dependencies(service_core dtb_quadruped_core_generate_messages_cpp)
