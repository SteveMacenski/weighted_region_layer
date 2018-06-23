cmake_minimum_required(VERSION 2.8.3)
project(weighted_region_layer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  map_msgs
  message_filters
  nav_msgs
  pcl_conversions
  pcl_ros
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  visualization_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
# )
## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
# )
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   map_msgs#   nav_msgs#   sensor_msgs#   std_msgs#   visualization_msgs
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES weighted_region_layer
  CATKIN_DEPENDS costmap_2d geometry_msgs map_msgs message_filters nav_msgs pcl_conversions pcl_ros pluginlib roscpp rospy sensor_msgs std_msgs std_srvs tf visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/weighted_region_layer.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h**"
)
