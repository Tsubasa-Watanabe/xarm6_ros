cmake_minimum_required(VERSION 2.8.3)
project(xarm6_gripper_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(stomp_origin src/stomp_origin.cpp)
target_link_libraries(stomp_origin ${catkin_LIBRARIES})
