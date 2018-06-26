cmake_minimum_required(VERSION 2.8.3)
project(weighted_region_layer)

add_compile_options(-std=c++17)

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
  rviz
  sensor_msgs
  std_msgs
  std_srvs
  tf
  visualization_msgs
  message_generation
)

if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt4_wrap_cpp(${ARGN})
  endmacro()
else()
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt5_wrap_cpp(${ARGN})
  endmacro()
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

find_package(Boost REQUIRED system filesystem) 

add_message_files(
  FILES
  data_serial.msg
)
add_service_files(
  FILES
  LoadWeightedRegionFile.srv
  SaveWeightedRegionFile.srv
)
generate_messages(
  DEPENDENCIES
  geometry_msgs  map_msgs   nav_msgs   sensor_msgs   std_msgs   visualization_msgs
)

catkin_package(
  LIBRARIES weighted_region_layer_costmap weighted_region_layer_rviz
  CATKIN_DEPENDS costmap_2d geometry_msgs map_msgs message_filters nav_msgs pcl_conversions pcl_ros pluginlib roscpp rospy sensor_msgs std_msgs std_srvs tf visualization_msgs message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

qt_wrap_cpp(MOC_FILES
  rviz_plugin/marking_tool.hpp
  rviz_plugin/marking_panel.hpp
)

set(SOURCE_FILES
  rviz_plugin/marking_tool.cpp
  rviz_plugin/marking_panel.cpp
  #${MOC_FILES}
)

add_library(${PROJECT_NAME}_costmap
  costmap_plugin/weighted_region_layer.cpp
)

add_dependencies(${PROJECT_NAME}_costmap ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_costmap
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME}_rviz
  ${SOURCE_FILES}
)

add_dependencies(${PROJECT_NAME}_rviz ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_rviz
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${rviz_DEFAULT_PLUGIN_LIBRARIES}
  ${QT_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_costmap ${PROJECT_NAME}_rviz
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  rviz_plugins.xml
  costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
