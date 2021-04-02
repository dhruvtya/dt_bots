cmake_minimum_required(VERSION 3.0.2)
project(dtb_quadruped_states)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)


# add_message_files(FILES )

 add_service_files(FILES QuadrupedBasicService.srv)

# add_action_files(FILES )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(basic_services src/basic_services.cpp)
target_link_libraries(basic_services ${catkin_LIBRARIES})
add_dependencies(basic_services dtb_quadruped_states_generate_messages_cpp)
