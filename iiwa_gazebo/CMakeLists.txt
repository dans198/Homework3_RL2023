cmake_minimum_required(VERSION 3.5)
project(iiwa_gazebo)

find_package(catkin REQUIRED
roscpp
sensor_msgs
std_msgs
message_generation
)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/iiwa_gazebo_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

foreach(dir launch worlds)
  install(DIRECTORY ${dir} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()
