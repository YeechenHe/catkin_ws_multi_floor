cmake_minimum_required(VERSION 3.0.2)
project(improved_global_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf
  tf2_geometry_msgs
  tf2_ros
)

################################################
## Dynamic reconfigure (generates ImprovedGlobalPlannerConfig.h)
################################################
generate_dynamic_reconfigure_options(
  cfg/ImprovedGlobalPlanner.cfg
)

###################################
## catkin package
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES improved_global_planner
  CATKIN_DEPENDS
    angles
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf
    tf2_ros
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(improved_global_planner
  src/quadratic_calculator.cpp
  src/dijkstra.cpp
  src/astar.cpp
  src/improved_astar.cpp
  src/grid_path.cpp
  src/gradient_path.cpp
  src/orientation_filter.cpp
  src/planner_core.cpp
)

add_dependencies(improved_global_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(improved_global_planner
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS improved_global_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES global_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
