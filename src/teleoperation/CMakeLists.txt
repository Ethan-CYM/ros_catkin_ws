cmake_minimum_required(VERSION 3.0.2)
project(teleoperation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
)

# find_package(ur_rtde REQUIRED PATHS
# "~/ur_rtde/ur_rtde/build/CMakeFiles/Export/lib/cmake/ur_rtde"
# "~/ur_rtde/ur_rtde/build/ur_rtde"
# )

 #catkin_python_setup()

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
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
   std_msgs
   trajectory_msgs
 )

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES teleoperation
 CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs trajectory_msgs trajectory_msgs
 DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ~/catkin_ws/devel/include
  # ~/ur_rtde/ur_rtde/include/ur_rtde
)

# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/teleoperation.cpp
# )


# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(touch_tel_ur src/touch_tel_ur.cpp)
target_link_libraries(touch_tel_ur ${catkin_LIBRARIES})
add_dependencies(touch_tel_ur ${catkin_EXPORTED_TARGETS})

add_executable(touch_tel_ur_rtde src/touch_tel_ur_rtde.cpp)
target_link_libraries(touch_tel_ur_rtde ${catkin_LIBRARIES})
add_dependencies(touch_tel_ur_rtde ${catkin_EXPORTED_TARGETS})

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# catkin_add_gtest(${PROJECT_NAME}-test test/test_teleoperation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# catkin_add_nosetests(test)
