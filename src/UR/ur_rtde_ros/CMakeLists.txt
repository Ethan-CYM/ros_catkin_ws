cmake_minimum_required(VERSION 3.5)
project(ur_rtde_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(ur_rtde REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  ur_rtde
)
LINK_DIRECTORIES(/usr/local/lib/)
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ur_rtde_ros
 CATKIN_DEPENDS roscpp rospy ur_rtde
 DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ur_rtde_ros.cpp
# )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(speedj src/speedj_example.cpp)
target_link_libraries(speedj ${ur_rtde_LIBRARIES} ${catkin_LIBRARIES} )
