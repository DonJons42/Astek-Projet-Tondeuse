cmake_minimum_required(VERSION 2.8.3)
project(vesc_diff_drive)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pluginlib
  roscpp
  nav_msgs
  std_msgs
  geometry_msgs
  tf
  vesc_msgs
  message_filters
)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS
    nodelet
    pluginlib
    roscpp
    nav_msgs
    std_msgs
    geometry_msgs
    tf
    vesc_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# node executable
add_executable(vesc_diff_drive_node src/vesc_diff_drive_node.cpp)
add_dependencies(vesc_diff_drive_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(vesc_diff_drive_node
  ${catkin_LIBRARIES}
)



#############
## Install ##
#############

install(TARGETS vesc_diff_drive_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#install(DIRECTORY include/${PROJECT_NAME}/
        #DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#############
## Testing ##
#############

# TODO
